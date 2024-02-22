package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Wrist {

    public Servo leftWrist;
    public Servo rightWrist;
    private double target;
    private double currentPos;
    private int time;
    private ElapsedTime timer = new ElapsedTime();
    private double moveIncrement;


    public Wrist(Servo leftJoint, Servo rightJoint) {
        this.leftWrist = leftJoint;
        this.rightWrist = rightJoint;
        this.rightWrist.setDirection(Servo.Direction.REVERSE);
        this.leftWrist.setPosition(RobotConstants.wrist_stowPos);
        this.rightWrist.setPosition(RobotConstants.wrist_stowPos);
        this.currentPos = RobotConstants.wrist_stowPos;
        this.target = this.currentPos;
    }

    public void setTarget(double input, int t) {
        if (input != this.currentPos) {
            this.target = input;
            this.time = t;
            this.timer.reset();
            this.moveIncrement = (input - this.currentPos) / t;
        }
    }
    public void moveWrist(double target) {

        // check for current encoder value of shoulder
        // convert above value to degree accounting for starting pos
        // move wrist to pos based on 90 - above degreee, will have to find out triangle details for the wrist

        this.leftWrist.setPosition(target);
        this.rightWrist.setPosition(target);
    }

    public void update(){
        double duration = this.timer.milliseconds();

        if (Math.abs(this.currentPos-this.target)>.01 && duration < this.time) {
            double temp = this.currentPos + duration * this.moveIncrement;
            this.leftWrist.setPosition(temp);
            this.rightWrist.setPosition(temp);
            //return temp;
        } else {
            this.leftWrist.setPosition(this.target);
            this.rightWrist.setPosition(this.target);
            this.currentPos = this.target;
        }
    }

    public double getTarget() {
        return this.target;
    }

    public boolean getServo(double tolerance) {
        return (Math.abs((this.rightWrist.getPosition() - this.currentPos)) < tolerance);
    }
}
