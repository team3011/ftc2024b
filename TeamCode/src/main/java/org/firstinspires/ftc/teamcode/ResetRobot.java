package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shoulder;


@TeleOp(name = "ResetRobot", group = "Robot")
public class ResetRobot extends LinearOpMode {
    Shoulder shoulder;
    @Override
    public void runOpMode() {
        Shoulder shoulder = new Shoulder(
                hardwareMap.get(DcMotorEx.class, "shoulder"),
                hardwareMap.get(TouchSensor.class, "shoulderSensor"));


        Servo servo = hardwareMap.get(Servo.class,"rightClaw");

        Lift lift = new Lift(
                hardwareMap.get(DcMotorEx.class,"lift"));
        waitForStart();
        shoulder.resetShoulder();
        double left_y = gamepad1.left_stick_y;
        double right_y = gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double right_x = gamepad1.right_stick_x;
        servo.setPosition(.5);
        while(opModeIsActive()) {
            left_y = zeroAnalogInput(gamepad1.left_stick_y);
            right_y = zeroAnalogInput(gamepad1.right_stick_y);
            left_x = zeroAnalogInput(gamepad1.left_stick_x);
            right_x = zeroAnalogInput(gamepad1.right_stick_x);
            lift.moveManual(right_y);
        }

    }

    private double zeroAnalogInput(double input){
        if (Math.abs(input)<0.1){
            input = 0;
        }
        return input;
    }
}

