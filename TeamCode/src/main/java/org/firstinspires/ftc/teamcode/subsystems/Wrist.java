package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    public Servo leftWrist;
    public Servo rightWrist;
    private double target;
    private double target2;


    public Wrist(Servo leftJoint, Servo rightJoint) {
        this.leftWrist = leftJoint;
        this.rightWrist = rightJoint;

        this.rightWrist.setDirection(Servo.Direction.REVERSE);
    }

    public void setTarget(double input) {
        this.target = input;
    }
    public void moveWrist(double target) {

        // check for current encoder value of shoulder
        // convert above value to degree accounting for starting pos
        // move wrist to pos based on 90 - above degreee, will have to find out triangle details for the wrist

        this.leftWrist.setPosition(target);
        this.rightWrist.setPosition(target);
    }

    public double getTarget() {
        return this.target;
    }
}
