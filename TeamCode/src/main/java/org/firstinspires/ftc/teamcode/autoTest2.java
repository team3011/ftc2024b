package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystemV2;
import org.firstinspires.ftc.teamcode.subsystems.cameraMaybe;

import java.util.List;

@Autonomous(name = "AutoTest2")
public class autoTest2 extends LinearOpMode {
    List<Recognition> currentRecognitions;

   // @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        NavxMicroNavigationSensor navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        RevBlinkinLedDriver blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "servo");
        RevBlinkinLedDriver.BlinkinPattern breathRed = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        RevBlinkinLedDriver.BlinkinPattern violet = RevBlinkinLedDriver.BlinkinPattern.VIOLET;

        Arm arm = new Arm(
                hardwareMap.get(DcMotorEx.class, "shoulder"),
                hardwareMap.get(TouchSensor.class, "shoulderSensor"),
                hardwareMap.get(DcMotorEx.class, "telescope"),
                hardwareMap.get(Servo.class, "leftClaw"),
                hardwareMap.get(Servo.class, "rightClaw"),
                hardwareMap.get(DcMotorEx.class, "lift"),
                hardwareMap.get(Servo.class, "left"),
                hardwareMap.get(Servo.class, "right"),
                navx,
                hardwareMap.get(TouchSensor.class, "clawSensor"),
                hardwareMap.get(RevBlinkinLedDriver.class, "servo")
        );
        DriveSystemV2 driveTrain = new DriveSystemV2(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                navx);
        cameraMaybe visionCam = new cameraMaybe(
                hardwareMap.get(WebcamName.class, "Webcam 1")
        );
        visionCam.initTfod();
        DcMotor ring = hardwareMap.get(DcMotor.class, "ring");


        ElapsedTime timer = new ElapsedTime();
        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navx.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        ring.setPower(0);
        arm.initialMove();

        waitForStart();
        //driveTrain.resetEncoder();

        double left_y = gamepad1.left_stick_y;
        double right_y = gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double right_x = gamepad1.right_stick_x;
        double left_t = gamepad1.left_trigger;
        double right_t = gamepad1.right_trigger;
        boolean a_state = false;
        boolean b_state = false;
        boolean x_state = false;
        boolean y_state = false;
        boolean dpadUp_state = false;
        boolean dpadDown_state = false;

        boolean shoulderWasMoving = false;
        boolean lifting = false;
        boolean autoRun = true;
        int autoRunStage = 0;
        int cameraDetect = 0;
        int step = 0;
        int half = 0;
        float objSize = 0;
        boolean autoTargetSet = false;


        //arm.tempWrist(.5);

        while (opModeIsActive()) {
            if (autoRun && autoRunStage == 0) {
                //move to the board
                if (!autoTargetSet) {
                    driveTrain.setTarget(-640, 1000,0,0);
                    autoTargetSet = true;
                    //prep for dropoff
                }
                if (driveTrain.update(25, 25)) {
                    autoRunStage = 1;
                    autoTargetSet = false;
                    //prep to stow
                    //this is the sleep in the prep to stow
                    Thread.sleep(1000);
                }
            }

            //driveTrain.update(50,50);
            telemetry.addData("x distance", driveTrain.getXDistance());
            telemetry.addData("y distance", driveTrain.getYDistance());
            telemetry.addData("x target", driveTrain.getxTarget());
            telemetry.addData("y target", driveTrain.getyTarget());
            telemetry.addData("x corr", driveTrain.getX_correction());
            telemetry.addData("y corr", driveTrain.getY_correction());
            telemetry.update();
            }
        }
}

