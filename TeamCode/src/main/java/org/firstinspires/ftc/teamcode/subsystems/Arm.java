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

    public Arm(DcMotorEx sm, TouchSensor st, DcMotorEx tm, Servo lc, Servo rc, DcMotorEx lm, Servo lw, Servo rw){
        this.shoulder = new Shoulder(sm, st);
        this.telescope = new Telescope(tm);
        this.claw = new JulliansClaw(lc, rc);
        this.lift = new Lift(lm);
        this.wrist = new Wrist(lw, rw);
    }

    public void manualMove(double input){
        double power = this.shoulder.moveManual(input);
        this.telescope.moveManual(power/3.33);
    }

    public void tempWrist(double input){
        this.wrist.moveWrist(input);
    }

    public void moveToStow(){
        this.shoulder.setPosition(RobotConstants.shoulder_stowPos);
        //this.telescope.setPosition(RobotConstants.telescope_stowPos);
        //this.wrist.moveWrist(RobotConstants.wrist_stowPos);
    }

    public void moveToPickup(){
        this.shoulder.setPosition(RobotConstants.shoulder_pickupPos);
        //this.telescope.setPosition(RobotConstants.telescope_stowPos);
        //this.wrist.moveWrist(RobotConstants.wrist_stowPos);
    }

    public void moveToDropOff(){
        this.shoulder.setPosition(RobotConstants.shoulder_dropOffPos);
        //this.telescope.setPosition(RobotConstants.telescope_stowPos);
        //this.wrist.moveWrist(RobotConstants.wrist_stowPos);
    }

    public double updateEverything(){
        return this.shoulder.update();
        //this.telescope.update();
    }

    public int shouldEncoder(){
        return this.shoulder.getEncoderValue();
    }

    public int telescopeEncoder(){
        return this.telescope.getEncoderValue();
    }
}
