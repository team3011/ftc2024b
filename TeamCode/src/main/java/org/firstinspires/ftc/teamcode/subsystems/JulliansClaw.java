package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class JulliansClaw {
    private Servo left;
    private Servo right;

    public <T> JulliansClaw(Servo leftClaw, Servo rightClaw) {
        this.left = leftClaw;
        this.right = rightClaw;
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

    public void partialBottom(){
        this.left.setPosition(RobotConstants.claw_partialBottom);
    }
    public void partialTop(){
        this.right.setPosition(RobotConstants.claw_partialTop);
    }
}
