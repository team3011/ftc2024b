package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Telescope;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystemV2;
import org.firstinspires.ftc.teamcode.subsystems.JulliansClaw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "TeleOppV1", group = "Robot")
public class TeleOppV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(
                hardwareMap.get(DcMotorEx.class,"shoulder"),
                hardwareMap.get(TouchSensor.class,"shoulderSensor"),
                hardwareMap.get(DcMotorEx.class,"telescope"),
                hardwareMap.get(Servo.class,"leftClaw"),
                hardwareMap.get(Servo.class,"rightClaw"),
                hardwareMap.get(DcMotorEx.class, "lift"),
                hardwareMap.get(Servo.class, "left"),
                hardwareMap.get(Servo.class, "right"));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        NavxMicroNavigationSensor navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        DriveSystemV2 driveTrain = new DriveSystemV2(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                navx,
                new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft")),
                new Encoder(hardwareMap.get(DcMotorEx.class, "backRight")));

        ElapsedTime timer = new ElapsedTime();
        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navx.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        waitForStart();
        //driveTrain.resetEncoder();

        double left_y = gamepad1.left_stick_y;
        double right_y = gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double right_x = gamepad1.right_stick_x;
        boolean a_state = false;
        boolean b_state = false;
        boolean x_state = false;
        boolean y_state = false;
        boolean shoulderWasMoving = false;




        while(opModeIsActive()){
            left_y = zeroAnalogInput(gamepad1.left_stick_y);
            right_y = zeroAnalogInput(gamepad1.right_stick_y);
            left_x = zeroAnalogInput(gamepad1.left_stick_x);
            right_x = zeroAnalogInput(gamepad1.right_stick_x);



            if (gamepad1.a && !a_state) {
                a_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.a && a_state) {
                a_state = false;
                arm.moveToPickup();
                //code here will fire when button released
            }
            if (gamepad1.x && !x_state) {
                x_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.x && x_state) {
                x_state = false;
                arm.moveToStow();
                //code here will fire when button released
            }
            if (gamepad1.y && !y_state) {
                y_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.y && y_state) {
                y_state = false;
                arm.moveToDropOff();
                //code here will fire when button released
            }

            if (left_y != 0) {
                shoulderWasMoving = true;
                arm.manualMove(left_y);
            } else if (shoulderWasMoving) {
                shoulderWasMoving = false;
                arm.manualMove(0);
            }

            double correction = arm.updateEverything();

            //driveTrain.drive(left_x,left_y,0);
            telemetry.addData("tele correction",correction);
            telemetry.addData("shoulder encoder",arm.shoulderEncoder());
            telemetry.addData("telescope encoder",arm.telescopeEncoder());
            telemetry.update();
        }
    }

    private double zeroAnalogInput(double input){
        if (Math.abs(input)<0.1){
            input = 0;
        }
        return input;
    }
}
