package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
import org.firstinspires.ftc.teamcode.subsystems.DriveSystemV2;
import org.firstinspires.ftc.teamcode.subsystems.cameraMaybe;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
@TeleOp(name = "TeleOpp", group = "Robot")
public class TeleOpp extends LinearOpMode {
    public static boolean driveEnabled = true;
    List<Recognition> currentRecognitions;
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        NavxMicroNavigationSensor navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        Arm arm = new Arm(
                hardwareMap.get(DcMotorEx.class,"shoulder"),
                hardwareMap.get(TouchSensor.class,"shoulderSensor"),
                hardwareMap.get(DcMotorEx.class,"telescope"),
                hardwareMap.get(Servo.class,"leftClaw"),
                hardwareMap.get(Servo.class,"rightClaw"),
                hardwareMap.get(DcMotorEx.class, "lift"),
                hardwareMap.get(Servo.class, "left"),
                hardwareMap.get(Servo.class, "right"),
                navx,
                hardwareMap.get(TouchSensor.class,"clawSensor"),
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
               // input,
                //input2
        );
        visionCam.initTfod();
        DcMotor ring = hardwareMap.get(DcMotor.class, "ring");


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
        double left_t = gamepad1.left_trigger;
        double right_t = gamepad1.right_trigger;
        boolean a_state = false;
        boolean b_state = false;
        boolean x_state = false;
        boolean y_state = false;
        boolean rightB_state = false;
        boolean dpadUp_state = false;
        boolean dpadDown_state = false;
        boolean dpadLeft_state = false;
        boolean dpadRight_state = false;
        int pickupLevel = 0;

        boolean shoulderWasMoving = false;
        boolean lifting = false;
        boolean autoRun = false;
        //int autoRunStage = 0;
        //boolean autoTargetSet = false;

        //arm.tempWrist(.5);

        boolean endGameStarted = false;
        arm.initialMove();
        while(opModeIsActive()){
            //ring.setPower(1);
            currentRecognitions = visionCam.returnRecogs();
            left_y = zeroAnalogInput(gamepad1.left_stick_y);
            right_y = zeroAnalogInput(gamepad1.right_stick_y);
            left_x = zeroAnalogInput(gamepad1.left_stick_x);
            right_x = zeroAnalogInput(gamepad1.right_stick_x);
            left_t = -zeroAnalogInput(gamepad1.left_trigger);
            right_t = zeroAnalogInput(gamepad1.right_trigger);



            if (gamepad1.a && !a_state) {
                a_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.a && a_state) {
                a_state = false;
                arm.moveToPickup(pickupLevel);
                //code here will fire when button released
            }
            if (gamepad1.b && !b_state) {
                b_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.b && b_state) {
                b_state = false;
                arm.resetShoulder();
                //code here will fire when button released
            }
            if (gamepad1.right_bumper && !rightB_state) {
                rightB_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.right_bumper && rightB_state) {
                rightB_state = false;
                pickupLevel += 1;
                if (pickupLevel > 3) {
                    pickupLevel = 0;
                }
                //code here will fire when button released
            }
            if (gamepad1.dpad_up && !dpadUp_state && TeleOpp.driveEnabled) {
                dpadUp_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.dpad_up && dpadUp_state && TeleOpp.driveEnabled) {
                dpadUp_state = false;
                driveTrain.setHeadingToMaintain(-1.57);
                //code here will fire when button released
            }
            if (gamepad1.dpad_down && !dpadDown_state && TeleOpp.driveEnabled) {
                dpadDown_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.dpad_down && dpadDown_state && TeleOpp.driveEnabled) {
                dpadDown_state = false;
                driveTrain.setHeadingToMaintain(1.57);
                //code here will fire when button released
            }
            if (gamepad1.dpad_left && !dpadLeft_state && TeleOpp.driveEnabled) {
                dpadLeft_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.dpad_left && dpadLeft_state && TeleOpp.driveEnabled) {
                dpadLeft_state = false;
                driveTrain.setHeadingToMaintain(0);
                //code here will fire when button released
            }
            if (gamepad1.dpad_right && !dpadRight_state && TeleOpp.driveEnabled) {
                dpadRight_state = true;
                //code here will fire when button pressed
            }
            if (!gamepad1.dpad_right && dpadRight_state && TeleOpp.driveEnabled) {
                dpadRight_state = false;
                driveTrain.setHeadingToMaintain(3.14);
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

            /*
            if (left_y != 0) {
                shoulderWasMoving = true;
                arm.manualMoveA(left_y);
            } else if (shoulderWasMoving) {
                shoulderWasMoving = false;
                arm.manualMoveA(0);
            }
             */

            int stage = 0;
            if (left_t != 0) {
                endGameStarted = true;
                shoulderWasMoving = true;
                stage = arm.prepForLift(left_t);
            } else if (shoulderWasMoving) {
                shoulderWasMoving = false;
                stage = arm.prepForLift(0);
            }

            if (right_t != 0) {
                lifting = true;
                arm.lifting(right_t);
            } else if (lifting) {
                lifting = false;
                arm.lifting(0);
            }

            //arm.manualMoveA(left_y);
            //arm.manualMoveB(right_y);
            if (!endGameStarted) {
                boolean correction = arm.updateEverything();
            }
            //if (TeleOpp.driveEnabled) {
            //    driveTrain.drive(left_x, left_y, 0);
            //}

            telemetry.addData("stage",stage);
            telemetry.addData("pickup level",pickupLevel);
            telemetry.addData("reset shoulder status",arm.getResetStatus());
            //telemetry.addData("right_y",right_y);
            //telemetry.addData("left_t",left_t);
            //telemetry.addData("sensor data",correction);
            telemetry.addData("shoulder encoder",arm.shoulderEncoder());
            telemetry.addData("telescope encoder",arm.telescopeEncoder());
            telemetry.addData("lift encoder",arm.liftEncoder());
            //telemetry.addData("shoulder current",arm.shoulderCurrent());
            //telemetry.addData("telescope current",arm.telescopeCurrent());
            //telemetry.addData("pitch",arm.getPitch());
            //telemetry.addData("x_distance",driveTrain.getXDistance());
            //telemetry.addData("y_distance",driveTrain.getYDistance());
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }   // end for() loop


            telemetry.update();

        }

        driveTrain.drive(0,0,0);
    }

    private double zeroAnalogInput(double input){
        if (Math.abs(input)<0.1){
            input = 0;
        } else if (input < 0) {
            input += .1;
        } else if (input > 0) {
            input -= .1;
        }
        return input;
    }
}
