package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class JulliansClaw {
    private Servo left;
    private Servo right;
    private TouchSensor sensor;

    public <T> JulliansClaw(Servo leftClaw, Servo rightClaw, TouchSensor s) {
        this.left = leftClaw;
        this.right = rightClaw;
        this.sensor = s;

    }

    public void openBottom(){
        this.left.setPosition(RobotConstants.claw_openBottom);
    }

    public void closeBottom(){
        this.left.setPosition(RobotConstants.claw_closeBottom);
    }

    public void openTop(){
        this.right.setPosition(RobotConstants.claw_openTop);
    }

    public void closeTop(){
        this.right.setPosition(RobotConstants.claw_closeTop);
    }

    public void closeTopHard(){
        this.right.setPosition(0.3);
    }

    public void partialBottom(){
        this.left.setPosition(RobotConstants.claw_partialBottom);
    }
    public void partialTop(){
        this.right.setPosition(RobotConstants.claw_partialTop);
    }

    public boolean getClawSensor(){
        return this.sensor.isPressed();
    }
}
