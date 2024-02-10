package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Arm {
    private Shoulder shoulder;
    private Telescope telescope;
    private JulliansClaw claw;
    private Lift lift;
    private Wrist wrist;
    private NavxMicroNavigationSensor navx;
    private IntegratingGyroscope gyro;
    private int state = -1;
    private int stage = 0;


    public Arm(DcMotorEx sm, TouchSensor st, DcMotorEx tm, Servo lc, Servo rc, DcMotorEx lm, Servo lw, Servo rw, NavxMicroNavigationSensor n){
        this.shoulder = new Shoulder(sm, st);
        this.telescope = new Telescope(tm);
        this.claw = new JulliansClaw(lc, rc);
        this.lift = new Lift(lm);
        this.wrist = new Wrist(lw, rw);
        this.navx = n;
        this.gyro = (IntegratingGyroscope)this.navx;
    }



    public void manualMoveA(double input){
        double speed = this.shoulder.moveManual(input);
        double current = this.shoulder.getCurrent();
        this.telescope.moveManual(speed/3.33);
        if (current>1400){
            double modifier = 1-(1700-current)/500.;
            this.lift.moveManual(-modifier);
        } else {
            this.lift.moveManual(0);
        }
    }
    public void manualMoveB(double input){
        //double power = this.shoulder.moveManual(input);
        //this.telescope.moveManual(power/3.33);
        //this.lift.moveManual(input);
        this.telescope.moveManual(input);
    }

    public int prepForLift(double input){
        input = input *.5;
        if (this.stage == 0 && this.telescope.getEncoderValue() > -300){
            this.telescope.moveManual(input);
            double current = this.telescope.getCurrent();
            if (current>600){
                double modifier = 1-(800-current)/200.;
                this.lift.moveManual(-modifier);
            } else {
                this.lift.moveManual(0);
            }
        } else if (this.stage == 0 && this.telescope.getEncoderValue() < -290) {
            this.stage = 1;
            this.lift.moveManual(0);
            this.telescope.moveManual(0);
            this.shoulder.moveManual(0);
        } else if (this.stage == 1 && this.shoulder.getEncoderValue() > -1355) {
            double speed = this.shoulder.moveManual(input);
            double current = this.shoulder.getCurrent();
            this.telescope.moveManual(speed/3.33);
            if (current>2250){
                double modifier = 1-(2550-current)/300.+.1;
                this.lift.moveManual(-modifier);
            } else {
                this.lift.moveManual(0);
            }
        } else if (this.stage == 1 && this.shoulder.getEncoderValue() < - 1350) {
            this.stage = 2;
            this.lift.moveManual(0);
            this.telescope.moveManual(0);
            this.shoulder.moveManual(0);
        } else if (this.stage == 2 && this.telescope.getEncoderValue() > -1700) {
            this.telescope.moveManual(input);
            double current = this.telescope.getCurrent();
            if (current > 600) {
                double modifier = 1 - (800 - current) / 200.;
                this.lift.moveManual(-modifier);
            } else {
                this.lift.moveManual(0);
            }
        } else if (this.stage == 2 && this.telescope.getEncoderValue() < -1690) {
            this.stage = 3;
            this.lift.moveManual(0);
            this.telescope.moveManual(0);
            this.shoulder.moveManual(0);
        } else if (this.stage == 3 && this.telescope.getEncoderValue() < -1500) {
            this.telescope.moveManual(-input);
        } else if (this.stage == 3 && this.telescope.getEncoderValue() > -1490) {
            this.stage = 4;
            this.lift.moveManual(0);
            this.telescope.moveManual(0);
            this.shoulder.moveManual(0);
        }
        return stage;


    }

    public void lifting(double input){
        input = input *.5;
        this.telescope.moveManual(input);
        double pitch = getPitch();
        if (Math.abs(input)>0 && pitch<-.01) {
            double modifier = input + (pitch/-.07)*.5;
            this.lift.moveManual(modifier);
        } else if (Math.abs(input)>0 && pitch > .01) {
            double modifier = input - (pitch/.07)*.5;
            this.lift.moveManual(modifier);
        } else {
            this.lift.moveManual(input);
        }
    }
    public void tempWrist(double input){
        this.wrist.moveWrist(input);
    }

    public double getPitch(){
        Orientation angles = this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotPitch = Math.toRadians(angles.secondAngle);
        return robotPitch;
    }

    public void initialMove(){
        this.telescope.resetEncoder();
        this.shoulder.setPosition(RobotConstants.shoulder_stowPos-150);
    }

    public void moveToStow() throws InterruptedException {
        if (this.state == -1) {
            this.claw.closeBottom();
            Thread.sleep(RobotConstants.claw_pickupPause);
            this.claw.closeTop();
            Thread.sleep(RobotConstants.claw_pickupPause);
        }
        if (this.state == 1) {
            this.claw.partialBottom();
            Thread.sleep(RobotConstants.claw_dropBottomPause);
            this.claw.partialTop();
            Thread.sleep(RobotConstants.claw_dropTopPause);
        }

        this.shoulder.setPosition(RobotConstants.shoulder_stowPos);
        this.telescope.setPosition(RobotConstants.telescope_stowPos);
        this.wrist.setTarget(RobotConstants.wrist_stowPos,RobotConstants.wrist_stowTime);
        //this.wrist.moveWrist(RobotConstants.wrist_stowPos);
    }

    public void moveToPickup(){
        this.shoulder.setPosition(RobotConstants.shoulder_pickupPos);
        this.telescope.setPosition(RobotConstants.telescope_pickupPos);
        this.wrist.setTarget(RobotConstants.wrist_pickupPos,RobotConstants.wrist_pickupTime);
        this.claw.openBottom();
        this.claw.openTop();
        this.state = -1;
    }

    public void moveToDropOff(){
        this.shoulder.setPosition(RobotConstants.shoulder_dropOffPos);
        this.telescope.setPosition(RobotConstants.telescope_dropOffHigh);
        this.wrist.setTarget(RobotConstants.wrist_dropOffPos,RobotConstants.wrist_dropOffTime);
        this.state = 1;
    }

    public double updateEverything(){

        this.shoulder.update();
        this.telescope.update();
        return this.wrist.update();
    }

    public int shoulderEncoder(){
        return this.shoulder.getEncoderValue();
    }

    public int telescopeEncoder(){
        return this.telescope.getEncoderValue();
    }

    public int liftEncoder() { return this.lift.getEncoderValue();}

    public double shoulderCurrent() { return this.shoulder.getCurrent();}
    public double telescopeCurrent() { return this.telescope.getCurrent();}
}
