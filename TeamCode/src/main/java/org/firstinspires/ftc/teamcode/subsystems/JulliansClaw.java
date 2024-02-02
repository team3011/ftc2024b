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
        this.left.setPosition(RobotConstants.julliansClawOpenBottom);
    }

    public void closeBottom(){
        this.left.setPosition(RobotConstants.julliansClawCloseBottom);
    }

    public void openTop(){
        this.right.setPosition(RobotConstants.julliansClawOpenTop);
    }

    public void closeTop(){
        this.right.setPosition(RobotConstants.julliansClawCloseTop);
    }
}
