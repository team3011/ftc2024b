package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Lift {
    DcMotorEx motor;
    private PIDCoefficients coeffs;
    private PIDFController controller;
    private int target = 0;
    private int lastTarget = 0;
    private MotionProfile profile;
    private ElapsedTime timer = new ElapsedTime();

    public Lift(DcMotorEx m) {
        this.motor = m;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.coeffs = new PIDCoefficients(RobotConstants.lift_kP,RobotConstants.lift_kI,RobotConstants.lift_kD);
        this.controller = new PIDFController(this.coeffs,0,0,0,(x, y)->RobotConstants.lift_kG);
        this.controller.setOutputBounds(-1,1);

        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0,0,0),
                new MotionState(0,0,0),
                RobotConstants.lift_maxVel,
                RobotConstants.lift_maxAccel,
                RobotConstants.lift_maxJerk
        );
    }

    public int getEncoderValue() {
        return this.motor.getCurrentPosition();
    }

    public void moveManual(double input){
        this.motor.setPower(input);
    }
}
