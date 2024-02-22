package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Telescope;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystemV2;
import org.firstinspires.ftc.teamcode.subsystems.JulliansClaw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.cameraMaybe;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystemV2;
import org.firstinspires.ftc.teamcode.subsystems.cameraMaybe;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "AutoTest")
public class autoTest extends LinearOpMode {
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

        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        ring.setPower(0);
        arm.initialMove();
        timer.reset();
        while (timer.seconds() < 3) {
            arm.updateEverything();
        }

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
        boolean autoRun = false;
        int autoRunStage = 0;
        int cameraDetect = 0;
        int step = 0;
        int half = 0;
        float objSize = 0;
        boolean autoTargetSet = false;
        boolean stop = false;


        //arm.tempWrist(.5);

        while (opModeIsActive()) {
            currentRecognitions = visionCam.returnRecogs();
            if (cameraDetect == 0 && autoRunStage == 0) {
                if (!autoTargetSet) {
                    driveTrain.setTarget(-690, 1050, 0, 0);
                    arm.moveToDropOffAuto();
                    ring.setPower(1);
                    autoTargetSet = true;
                }
                driveTrain.update(50, 50);
                if (currentRecognitions.size() != 0) {
                    telemetry.addData("# Objects Detected", currentRecognitions.size());

                    // Step through the list of recognitions and display info for each one.
                    for (Recognition recognition : currentRecognitions) {
                        double x = (recognition.getLeft() + recognition.getRight()) / 2;
                        double y = (recognition.getTop() + recognition.getBottom()) / 2;

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position", "%.0f / %.0f", x, y);
                        telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    }   // end for() loop
                    telemetry.addData("cam", cameraDetect);
                    telemetry.addData("stage", autoRunStage);
                    telemetry.update();
                    Recognition recognition = currentRecognitions.get(0);
                    objSize = recognition.getWidth() * recognition.getHeight();
                    if (objSize < (70 * 70)) {
                        cameraDetect = 1;
                        if (driveTrain.update(20, 20)) {
                            autoRunStage = 1;
                            autoTargetSet = false;
                        }
                    } else if (objSize >= (70 * 70) && objSize <= (90 * 90)) {
                        cameraDetect = 2;
                        if (driveTrain.update(20, 20)) {
                            autoRunStage = 1;
                            autoTargetSet = false;
                        }
                    } else {
                        cameraDetect = 3;
                        if (driveTrain.update(20, 20)) {
                            autoRunStage = 1;
                            autoTargetSet = false;
                        }
                    }
                }
            } // end of auto stage 0


            // auto stage 1
            // place pixel on backboard & floor based on detect TSE location
            else if (cameraDetect == 1 && autoRunStage == 1) {
                if (step == 0) {
                    if (!autoTargetSet) {
                        driveTrain.setTarget(-850, 1050, 0, 0);
                        autoTargetSet = true;
                    }
                    // move left on backboard to desired location
                    if (driveTrain.update(25, 25)) {
                        arm.moveToStowSingle();
                        autoTargetSet = false;
                        step = 1;
                        // check if we there, if yes move to next step
                    }
                }
                if (step == 1) {
                    if (!autoTargetSet) {
                        driveTrain.setTarget(-740, 225, 0, 0);
                        arm.moveToPickupClose(0);
                        autoTargetSet = true;
                    }
                    // move to far floor location
                    if (driveTrain.update(20, 20)) {
                        // drop remaining pixel
                        autoTargetSet = false;
                        step = 2;
                    }
                }
                if (step == 2 && arm.shoulderEncoder() < 10) {
                    autoRunStage = 2;
                    step = 0;
                    arm.moveToStowDouble();
                    stop = true;
                }

            }
            else if (cameraDetect == 2 && autoRunStage == 1) {
                if (step == 0) {
                    arm.moveToStowSingle();
                    step = 1;
                }
                if (step == 1) {
                    if (!autoTargetSet) {
                        driveTrain.setTarget(-930, 550, 0, 0);
                        arm.moveToPickupClose(0);
                        autoTargetSet = true;
                    }
                    // move to far floor location
                    if (driveTrain.update(20, 20)) {
                        // drop remaining pixel
                        autoTargetSet = false;
                        step = 2;
                    }
                }
                if (step == 2 && arm.shoulderEncoder() < 10) {
                    autoRunStage = 2;
                    step = 0;
                    arm.moveToStowDouble();
                    stop = true;
                }

            }
            else if (cameraDetect == 3 && autoRunStage == 1) {
                if (step == 0) {
                    if (!autoTargetSet) {
                        driveTrain.setTarget(-530, 1050, 0, 0);
                        autoTargetSet = true;
                    }
                    // move left on backboard to desired location
                    if (driveTrain.update(25, 25)) {
                        arm.moveToStowSingle();
                        autoTargetSet = false;
                        step = 1;
                        // check if we there, if yes move to next step
                    }
                }
                if (step == 1) {
                    if (!autoTargetSet) {
                        driveTrain.setTarget(-710, 775, 0, 0);
                        arm.moveToPickupClose(0);
                        autoTargetSet = true;
                    }
                    // move to far floor location
                    if (driveTrain.update(20, 20)) {
                        // drop remaining pixel
                        autoTargetSet = false;
                        step = 2;
                    }
                }
                if (step == 2 && arm.shoulderEncoder() < 1 && arm.returnWrist(0.005)) {
                    autoRunStage = 2;
                    step = 0;
                    arm.moveToStowDouble();
                    stop = true;
                }

            }


            // end of auto stage 1
            arm.updateEverything();
            if (stop) {
                driveTrain.drive(0, 0, 0);
            }
            telemetry.addData("shoulder", arm.shoulderEncoder());
            telemetry.addData("retwrist", arm.returnWrist(0.01));
            telemetry.addData("cam", cameraDetect);
            telemetry.addData("stage", autoRunStage);
            telemetry.addData("step", step);
            telemetry.addData("x distance", driveTrain.getXDistance());
            telemetry.addData("y distance", driveTrain.getYDistance());
            telemetry.addData("x target", driveTrain.getxTarget());
            telemetry.addData("y target", driveTrain.getyTarget());
            telemetry.addData("x corr", driveTrain.getX_correction());
            telemetry.addData("y corr", driveTrain.getY_correction());
            telemetry.addData("should we move", !autoTargetSet);
            telemetry.update();
        }
    }
}


