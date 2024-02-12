package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Shoulder {
    private DcMotorEx motor;
    private TouchSensor touch;
    private PIDCoefficients coeffs;
    private PIDFController controller;
    private int target = 0;
    private int lastTarget = 0;
    private MotionProfile profile;
    private ElapsedTime timer = new ElapsedTime();

    private double power = 0;
    private boolean manualMoving = false;
    private boolean resetEncoder = false;

    public Shoulder(DcMotorEx m, TouchSensor t) {
        this.motor = m;
        this.touch = t;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.coeffs = new PIDCoefficients(RobotConstants.shoulder_kP,RobotConstants.shoulder_kI,RobotConstants.shoulder_kD);
        this.controller = new PIDFController(this.coeffs,0,0,0,(x,y)->RobotConstants.shoulder_kG);
        this.controller.setOutputBounds(-1,1);

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0,0,0),
                new MotionState(0,0,0),
                RobotConstants.shoulder_maxVel,
                RobotConstants.shoulder_maxAccel,
                RobotConstants.shoulder_maxJerk
        );
    }

    //up is encoder value negative
    public int getEncoderValue() {
        return this.motor.getCurrentPosition();
    }

    public boolean getTouchValue() {
        return this.touch.isPressed();
    }

    //input > 0 shoulder goes down
    public double moveManual(double input){
        this.manualMoving = true;
        if (this.touch.isPressed() && input > 0) {
            this.motor.setPower(0);
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            this.motor.setPower(input);
        }
        setPosition(this.motor.getCurrentPosition());
        return this.motor.getPower();
    }

    public void resetShoulder(){
        this.motor.setPower(.15);
        while (!this.touch.isPressed()) {}
        this.motor.setPower(0);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.lastTarget = this.motor.getCurrentPosition();
    }

    //@param target - encoder counts 0 to negative numbers
    public void setPosition(int t){
        if (t == 0) {
            this.resetEncoder = true;
        }
        //convert centimeters to ticks
        this.target = t;
        if (this.target != this.lastTarget) {
            this.coeffs = new PIDCoefficients(RobotConstants.shoulder_kP,RobotConstants.shoulder_kI,RobotConstants.shoulder_kD);
            this.controller = new PIDFController(this.coeffs,0,0,0,(x,y)->RobotConstants.shoulder_kG);
            this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(this.motor.getCurrentPosition(),0,0),
                    new MotionState(this.target,0,0),
                    RobotConstants.shoulder_maxVel,
                    RobotConstants.shoulder_maxAccel,
                    RobotConstants.shoulder_maxJerk
            );
            this.lastTarget = this.target;
            this.timer.reset();
        }
    }

    public int getTarget(){
        return this.target;
    }

    public boolean getResetEncoder() { return this.resetEncoder;}

    public double update() {
        double correction = 0;
        if (!this.manualMoving) {
            int motor_Pos = this.motor.getCurrentPosition();
            if (this.resetEncoder && Math.abs(motor_Pos) < 10 && !this.touch.isPressed()) {
                this.motor.setPower(.2);
            } else if (this.resetEncoder && this.touch.isPressed()) {
                this.motor.setPower(0);
                this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.lastTarget = 0;
                this.target = 0;
                this.resetEncoder = false;
            } else {
                MotionState state = this.profile.get(this.timer.seconds());
                this.controller.setTargetPosition(state.getX());
                this.controller.setTargetVelocity(state.getV());
                this.controller.setTargetAcceleration(state.getA());
                correction = controller.update(motor_Pos);

                if (this.resetEncoder && motor_Pos > -1500) {
                    correction *= .2;
                }
                if (!this.resetEncoder && motor_Pos < -1500 && correction < 0) {
                    correction *= .2;
                }
                if (this.touch.isPressed() && correction > 0) {
                    this.motor.setPower(0);
                    this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    this.lastTarget = 0;
                    this.target = 0;
                    this.resetEncoder = false;
                } else {
                    this.motor.setPower(correction);
                }
            }
        }
        this.manualMoving = false;
        return correction;
    }

    public double getCurrent(){
        return this.motor.getCurrent(CurrentUnit.MILLIAMPS);
    }
}
