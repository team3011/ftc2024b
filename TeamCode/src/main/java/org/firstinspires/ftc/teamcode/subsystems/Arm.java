package org.firstinspires.ftc.teamcode.subsystems;

import android.text.method.Touch;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.Timer;

@Config
public class Arm {
    private Shoulder shoulder;
    private Telescope telescope;
    private JulliansClaw claw;
    private Lift lift;
    private Wrist wrist;
    private NavxMicroNavigationSensor navx;
    private IntegratingGyroscope gyro;
    private int state = 0;
    private int stage = -3;
    public static int shoulderOffset = 0;
    public static double wristOffset = 0;
    private boolean autoPickup = false;

    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern breathRed = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
    private RevBlinkinLedDriver.BlinkinPattern violet = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private ElapsedTime colorTimer = new ElapsedTime();

    public Arm(DcMotorEx sm, TouchSensor st, DcMotorEx tm, Servo lc, Servo rc, DcMotorEx lm, Servo lw, Servo rw, NavxMicroNavigationSensor n, TouchSensor wt, RevBlinkinLedDriver rbld){
        this.shoulder = new Shoulder(sm, st);
        this.telescope = new Telescope(tm);
        this.claw = new JulliansClaw(lc, rc, wt);
        this.lift = new Lift(lm);
        this.wrist = new Wrist(lw, rw);
        this.navx = n;
        this.gyro = (IntegratingGyroscope)this.navx;
        this.blinkinLedDriver = rbld;
        this.blinkinLedDriver.setPattern(breathRed);
    }

    public void manualMoveA(double input){
        this.shoulder.moveManual(input*.5);
        //double speed = this.shoulder.moveManual(input);
        //double current = this.shoulder.getCurrent();
        //this.telescope.moveManual(speed/3.33);
        //if (current>1400){
        //    double modifier = 1-(1700-current)/500.;
        //    this.lift.moveManual(-modifier);
        //} else {
        //    this.lift.moveManual(0);
       // }
    }
    public void manualMoveB(double input){
        //double power = this.shoulder.moveManual(input);
        //this.telescope.moveManual(power/3.33);
        //this.lift.moveManual(input);
        this.telescope.moveManual(input*.5);
    }

    public int prepForLift(double input){
        input = input *.6;
        //move shoulder up to -100
        if (this.stage == -3 && this.shoulder.getEncoderValue() > -100) {
            wrist.moveWrist(RobotConstants.wrist_temp);
            if (this.shoulder.getEncoderValue() > -100) {
                this.shoulder.moveManual(input);
            } else {
                this.shoulder.moveManual(0);
            }
        }else if (this.stage == -3 && this.shoulder.getEncoderValue() <-100) {
            this.shoulder.moveManual(0);
            this.stage = -2;
        //move telescope in to 0
        }else if (this.stage == -2 && this.telescope.getEncoderValue() < 0) {
            if (this.telescope.getEncoderValue() < 0) {
                this.telescope.moveManual(-input);
            }
        }else if (this.stage == -2) {
            this.stage = -1;
            this.telescope.moveManual(0);
        //move shoulder to 0
        } else if (this.stage == -1 && this.shoulder.getEncoderValue() < 0) {
            this.shoulder.moveManual(-input);
        } else if (this.stage == -1){
            this.stage = 0;
            this.shoulder.moveManual(0);
        //push telescope out to engage cable
        } else if (this.stage == 0 && this.telescope.getEncoderValue() > -300){
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
        //move shoulder into position to lift
        } else if (this.stage == 1 && this.shoulder.getEncoderValue() > -1400) {
            double speed = this.shoulder.moveManual(input);
            double current = this.shoulder.getCurrent();
            this.telescope.moveManual(speed/3.33);
            if (current>2250){
                double modifier = 1-(2550-current)/300.+.1;
                this.lift.moveManual(-modifier);
            } else {
                this.lift.moveManual(0);
            }
        } else if (this.stage == 1 && this.shoulder.getEncoderValue() < - 1390) {
            this.stage = 2;
            this.lift.moveManual(0);
            this.telescope.moveManual(0);
            this.shoulder.moveManual(0);
        //move telescope up to above bar
        } else if (this.stage == 2 && this.telescope.getEncoderValue() > -2200) {
            if (this.shoulder.getEncoderValue() > -1400) {
                this.shoulder.moveManual(input*1.2);
            } else {
                this.shoulder.moveManual(0);
            }
            double currentS = this.shoulder.getCurrent();
            this.telescope.moveManual(input);
            double currentT = this.telescope.getCurrent();
            if (currentS>2250) {
                double modifier = 1 - (2550 - currentS) / 300. + .1;
                this.lift.moveManual(-modifier);
            } else if (currentT > 600) {
                double modifier = 1 - (800 - currentT) / 200.;
                this.lift.moveManual(-modifier);
            } else {
                this.lift.moveManual(0);
            }
        } else if (this.stage == 2 && this.telescope.getEncoderValue() < -2190) {
            this.stage = 3;
            this.lift.moveManual(0);
            this.telescope.moveManual(0);
            this.shoulder.moveManual(0);
        //retract the telescope to slack the line
        } else if (this.stage == 3 && this.telescope.getEncoderValue() < -1900) {
            this.telescope.moveManual(-input*.5);
        } else if (this.stage == 3 && this.telescope.getEncoderValue() > -1890) {
            this.stage = 4;
            this.lift.moveManual(0);
            this.telescope.moveManual(0);
            this.shoulder.moveManual(0);
        }
        return stage;


    }

    public void lifting(double input){
        input = input *.5;
        double pitch = getPitch();
        if (Math.abs(input)>0 && pitch<-.01) {
            double modifier = input + (pitch/-.07)*.5;
            this.lift.moveManual(modifier);
            if (this.telescope.getEncoderValue()>-350){
                this.telescope.moveManual(0);
            }
        } else if (Math.abs(input)>0 && pitch > .01) {
            double modifier = input - (pitch/.07)*.5;
            this.lift.moveManual(modifier);
            if (this.telescope.getEncoderValue()>-350){
                this.telescope.moveManual(0);
            }
        } else {
            this.lift.moveManual(input);
            if (this.telescope.getEncoderValue()>-350){
                this.telescope.moveManual(0);
            } else {
                this.telescope.moveManual(input);
            }
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
        this.claw.closeTop();
        this.claw.closeBottom();
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
        this.state = 0;
    }

    public void moveToStowSingle() throws InterruptedException {
        this.claw.partialBottom();
        Thread.sleep(RobotConstants.claw_dropBottomPause);
        this.shoulder.setPosition(RobotConstants.shoulder_stowPos);
        this.telescope.setPosition(RobotConstants.telescope_stowPos);
        this.wrist.setTarget(RobotConstants.wrist_stowPos,RobotConstants.wrist_stowTime);
    }
    public void moveToStowDouble() throws InterruptedException {
        this.claw.openTop();
        this.claw.openBottom();
        Thread.sleep(RobotConstants.claw_dropBottomPause);
        this.shoulder.setPosition(RobotConstants.shoulder_stowPos);
        this.telescope.setPosition(RobotConstants.telescope_stowPos);
        this.wrist.setTarget(RobotConstants.wrist_stowPos,250);
        this.state = 0;
    }

    public void moveToPickup(int level){
        if (this.state != 1) {
            if (level == 0) {
                this.shoulder.setPosition(RobotConstants.shoulder_pickupPos);
                this.wrist.setTarget(RobotConstants.wrist_pickupPos, RobotConstants.wrist_pickupTime);
            } else if (level == 1) {
                this.shoulder.setPosition(RobotConstants.shoulder_pickupPos + RobotConstants.shoulder_stack1);
                this.wrist.setTarget(RobotConstants.wrist_pickupPos, RobotConstants.wrist_pickupTime);
            } else if (level == 2) {
                this.shoulder.setPosition(RobotConstants.shoulder_pickupPos + RobotConstants.shoulder_stack2);
                this.wrist.setTarget(RobotConstants.wrist_pickupPos, RobotConstants.wrist_pickupTime);
            } else if (level == 3) {
                this.shoulder.setPosition(RobotConstants.shoulder_pickupPos + RobotConstants.shoulder_stack3);
                this.wrist.setTarget(RobotConstants.wrist_pickupPos + Arm.wristOffset, RobotConstants.wrist_pickupTime);
            }
            this.telescope.setPosition(RobotConstants.telescope_pickupPos);
            this.claw.openBottom();
            this.claw.openTop();
            this.state = -1;
            this.autoPickup = true;

        }
    }

    public void moveToPickupClose(){
        this.shoulder.setPosition(RobotConstants.shoulder_pickupPos);
        this.wrist.setTarget(RobotConstants.wrist_pickupPos, RobotConstants.wrist_pickupTime);
        this.telescope.setPosition(RobotConstants.telescope_pickupPos);
        this.claw.closeBottom();
        this.claw.closeTopHard();
        this.autoPickup = false;
        this.state = -1;
    }

    public void moveToDropOff(){
        if (this.state != -1) {
            this.shoulder.setPosition(RobotConstants.shoulder_dropOffPos);
            this.telescope.setPosition(RobotConstants.telescope_dropOffHigh);
            this.wrist.setTarget(RobotConstants.wrist_dropOffPos, RobotConstants.wrist_dropOffTime);
            this.state = 1;
        }
    }

    public void moveToDropOffAuto(){
        this.shoulder.setPosition(RobotConstants.shoulder_dropOffPos);
        this.telescope.setPosition(RobotConstants.telescope_dropOffHigh+1500);
        this.wrist.setTarget(RobotConstants.wrist_dropOffPos, RobotConstants.wrist_dropOffTime);
    }

    public boolean getClawSensor() {
        this.autoPickup = true;
        return this.getClawSensor();
    }

    public boolean updateEverything() throws InterruptedException {
        this.wrist.update();
        this.shoulder.update();
        if (this.autoPickup && this.claw.getClawSensor()) {
            moveToStow();
            this.autoPickup = false;
            this.blinkinLedDriver.setPattern(this.violet);
            this.colorTimer.reset();
        }
        if (this.colorTimer.seconds()>2){
            this.blinkinLedDriver.setPattern(this.breathRed);
        }
        this.telescope.update();
        return this.claw.getClawSensor();
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

    public void resetShoulder() { this.shoulder.resetShoulder();}

    public boolean getResetStatus() {
        return this.shoulder.getResetEncoder();
    }
    public boolean returnWrist(double tolerance) {
        return this.wrist.getServo(tolerance);
    }
}
