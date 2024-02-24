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

@Autonomous(name = "AutoBlue")
public class AutoBlue extends LinearOpMode {
    List<Recognition> currentRecognitions;

    // @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        NavxMicroNavigationSensor navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        RevBlinkinLedDriver blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "servo");
        RevBlinkinLedDriver.BlinkinPattern breathRed = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
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
                hardwareMap.get(RevBlinkinLedDriver.class, "servo"),
                false
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

        blinkinLedDriver.setPattern(breathRed);
        ring.setPower(0);
        arm.initialMove();
        timer.reset();
        while (timer.seconds() < 3) {
            arm.updateEverything();
        }

        waitForStart();
        //driveTrain.resetEncoder();

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
        boolean armUpdateOverride = false;
        int lastY = 0;


        ElapsedTime resetTimer = new ElapsedTime();
        boolean startTimer = false;
        while (opModeIsActive()) {
            if (!startTimer) {
                resetTimer.reset();
                startTimer = true;
            }
            //move to the board, put arm to dropoff
            if (autoRunStage == 0) {
                if (!autoTargetSet) {
                    driveTrain.setTarget(950, 1010, 0, 0);
                    RobotConstants.shoulder_kP = .01;
                    arm.moveToDropOffAuto();
                    ring.setPower(1);
                    autoTargetSet = true;
                }
                if(driveTrain.update(20,20)){
                    autoRunStage = 1;
                    autoTargetSet = false;
                }
                //turn on camera to detect objects
            } else if (autoRunStage == 1) {
                if (cameraDetect == 0) {
                    currentRecognitions = visionCam.returnRecogs();
                }
                //we have found an object
                if (currentRecognitions.size() != 0) {
                    Recognition recognition = currentRecognitions.get(0);
                    objSize = recognition.getWidth() * recognition.getHeight();
                    telemetry.addData("counts", currentRecognitions.size());
                    telemetry.addData("size", objSize);
                    telemetry.addData("width", recognition.getWidth());
                    telemetry.addData("height", recognition.getHeight());
                    //left
                    if (objSize < (70 * 70)) {
                        cameraDetect = 3;
                        autoRunStage = 2;
                        //middle
                    } else if (objSize >= (70 * 70) && objSize <= (90 * 90)) {
                        cameraDetect = 2;
                        autoRunStage = 3;
                        //right
                    } else {
                        cameraDetect = 1;
                        autoRunStage = 2;
                    }
                }
                //move to location on board
            } else if (autoRunStage == 2) {
                ring.setPower(0);
                if (cameraDetect == 1) {
                    if(!autoTargetSet) {
                        driveTrain.setTarget(450, 1005, 0, 0);
                        autoTargetSet = true;
                    }

                } else if (cameraDetect == 2){
                    if(!autoTargetSet) {
                        driveTrain.setTarget(690, 1005, 0, 0);
                        autoTargetSet = true;
                    }
                } else if (cameraDetect == 3){
                    if(!autoTargetSet) {
                        driveTrain.setTarget(880, 1005, 0, 0);
                        autoTargetSet = true;
                    }
                }
                if (driveTrain.update(50,50)) {
                    autoTargetSet = false;
                    autoRunStage = 3;
                }
                //drop off bottom pixel
            } else if (autoRunStage == 3) {
                arm.moveToStowSingle();
                autoTargetSet = false;
                autoRunStage = 4;
                //move to drop of pixel on floor
            } else if (autoRunStage == 4) {
                if (!autoTargetSet) {
                    arm.moveToPickupClose();
                    if (cameraDetect == 1) {
                        driveTrain.setTarget(710, 775, 0, 0);
                        lastY = 775;
                    } else if (cameraDetect == 2) {
                        driveTrain.setTarget(970, 600, 0, 0);
                        lastY = 580;
                    } else {
                        driveTrain.setTarget(710, 225, 0, 0);
                        lastY = 225;
                    }
                    autoTargetSet = true;
                } else if (driveTrain.update(50, 50) && arm.shoulderEncoder()>-60) {
                    autoRunStage = 5;
                    arm.moveToStowDouble();
                    autoTargetSet = false;
                }
                //move to lane 3
            } else if (autoRunStage == 5) {
                if (!autoTargetSet) {
                    driveTrain.setTarget(1330, lastY+100, 0, 0);
                    autoTargetSet = true;
                } else if (driveTrain.update(20, 20)) {
                    autoRunStage = 6;
                    autoTargetSet = false;
                }
                //move to stack
            } else if (autoRunStage == 6) {
                if (!autoTargetSet) {
                    driveTrain.setTarget(1285, -1520, 0, 0);
                    arm.moveToPickup(3);
                    autoTargetSet = true;
                } else if (driveTrain.update(20,20)) {
                    driveTrain.drive(0, 0, 0);
                    armUpdateOverride = true;
                    if (arm.shoulderEncoder()<-85){
                        arm.manualMoveA(.4);
                    } else {
                        arm.manualMoveA(0);
                        if (arm.getClawSensor()) {
                            armUpdateOverride = false;
                            autoRunStage = 7;
                            autoTargetSet = false;
                        }
                    }
                }
                //return back to top of lane 3
            } else if (autoRunStage == 7) {
                armUpdateOverride = false;
                if (!autoTargetSet) {
                    driveTrain.setTarget(1320, 775, 0, 0);
                    autoTargetSet = true;
                } else if (driveTrain.update(20,20)) {
                    autoRunStage = 8;
                    autoTargetSet = false;
                }
                //move to backboard
            } else if (autoRunStage == 8) {
                if (!autoTargetSet) {
                    driveTrain.setTarget(690, 1000, 0, 0);
                    arm.moveToDropOffAuto();
                    autoTargetSet = true;
                } else if (driveTrain.update(20,20)) {
                    autoRunStage = 9;
                    autoTargetSet = false;
                    arm.moveToStow();
                    driveTrain.drive(0, 0, 0);
                }
            }

            if (autoRunStage < 8 && resetTimer.seconds()>25){
                arm.moveToStow();
                driveTrain.drive(0, 0, 0);
            }

            telemetry.addData("x distance", driveTrain.getXDistance());
            telemetry.addData("y distance", driveTrain.getYDistance());
            telemetry.addData("x target", driveTrain.getxTarget());
            telemetry.addData("y target", driveTrain.getyTarget());
            telemetry.addData("cam", cameraDetect);
            telemetry.addData("stage", autoRunStage);
            telemetry.addData("shoulder", arm.shoulderEncoder());

            if (!armUpdateOverride) {
                arm.updateEverything();
            }
            if (stop) {
                driveTrain.drive(0, 0, 0);
            }

            //telemetry.addData("shoulder", arm.shoulderEncoder());
            //telemetry.addData("retwrist", arm.returnWrist(0.01));
            //telemetry.addData("cam", cameraDetect);
            //telemetry.addData("stage", autoRunStage);
            //telemetry.addData("step", step);
            //telemetry.addData("x distance", driveTrain.getXDistance());
            //telemetry.addData("y distance", driveTrain.getYDistance());
            //telemetry.addData("x target", driveTrain.getxTarget());
            //telemetry.addData("y target", driveTrain.getyTarget());
            //telemetry.addData("x corr", driveTrain.getX_correction());
            //telemetry.addData("y corr", driveTrain.getY_correction());
            //telemetry.addData("should we move", !autoTargetSet);
            telemetry.update();
        }
    }
}



