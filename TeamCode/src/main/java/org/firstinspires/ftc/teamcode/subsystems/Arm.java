package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Arm {
    private Shoulder shoulder;
    private Telescope telescope;
    private JulliansClaw claw;
    private Lift lift;
    private Wrist wrist;
    private int state = -1;


    public Arm(DcMotorEx sm, TouchSensor st, DcMotorEx tm, Servo lc, Servo rc, DcMotorEx lm, Servo lw, Servo rw){
        this.shoulder = new Shoulder(sm, st);
        this.telescope = new Telescope(tm);
        this.claw = new JulliansClaw(lc, rc);
        this.lift = new Lift(lm);
        this.wrist = new Wrist(lw, rw);
    }



    public void manualMoveA(double input){
        double power = this.shoulder.moveManual(input);
        this.telescope.moveManual(power/3.33);
        //this.lift.moveManual(input);
    }
    public void manualMoveB(double input){
        //double power = this.shoulder.moveManual(input);
        //this.telescope.moveManual(power/3.33);
        this.lift.moveManual(input);
    }

    public void tempWrist(double input){
        this.wrist.moveWrist(input);
    }

    public void moveToStow() throws InterruptedException {
        if (this.state == -1) {
            this.claw.closeBottom();
            Thread.sleep(RobotConstants.claw_pickupPause);
            this.claw.closeTop();
            Thread.sleep(RobotConstants.claw_pickupPause);
        }
        if (this.state == 1) {
            this.claw.partialTop();
            Thread.sleep(RobotConstants.claw_dropTopPause);
            this.claw.partialBottom();
            Thread.sleep(RobotConstants.claw_dropBottomPause);
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
}
