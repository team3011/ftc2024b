package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.Shoulder;


@TeleOp(name = "ResetRobot", group = "Robot")
public class ResetRobot extends LinearOpMode {
    Shoulder shoulder;
    @Override
    public void runOpMode() {
        Shoulder shoulder = new Shoulder(
                hardwareMap.get(DcMotorEx.class, "shoulder"),
                hardwareMap.get(TouchSensor.class, "shoulderSensor"));

        waitForStart();
        shoulder.resetShoulder();
    }
}

