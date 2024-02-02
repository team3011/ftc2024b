package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.ArmV2;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystemV2;
import org.firstinspires.ftc.teamcode.subsystems.JulliansClaw;
import org.firstinspires.ftc.teamcode.subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "TeleOppV1", group = "Robot")
public class TeleOppV1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shoulder shoulder = new Shoulder(
                hardwareMap.get(DcMotorEx.class,"shoulder"),
                hardwareMap.get(TouchSensor.class,"shoulderSensor"));
        ArmV2 arm = new ArmV2(
                hardwareMap.get(DcMotorEx.class,"telescope"));
        JulliansClaw claw = new JulliansClaw(
                hardwareMap.get(Servo.class,"leftClaw"),
                hardwareMap.get(Servo.class,"rightClaw"));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        //AHRS navx = AHRS.getInstance(
        //        hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
        //        AHRS.DeviceDataType.kProcessedData);

        /*
        DriveSystemV2 driveTrain = new DriveSystemV2(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                navx,
                new Encoder(hardwareMap.get(DcMotorEx.class, "backLeft")),
                new Encoder(hardwareMap.get(DcMotorEx.class, "backRight")));
        */
        //Lift lift = new Lift(
        //        hardwareMap.get(DcMotorEx.class, "lift"));
        Wrist wrist  = new Wrist(
                hardwareMap.get(Servo.class, "left"),
                hardwareMap.get(Servo.class, "right"));
        double wposition = 1.0;

        //while (navx.isCalibrating()){
        //    telemetry.addData("navx calibation...",navx.isCalibrating());
        //    telemetry.update();
        //}


        waitForStart();

        //arm.setPosition(RobotConstants.arm_minPos);
        //driveTrain.resetEncoder();

        // Scan servo till stop pressed.
        float left_y = gamepad1.left_stick_y;
        float right_y = gamepad1.right_stick_y;
        float left_x = gamepad1.left_stick_x;
        float right_x = gamepad1.right_stick_x;
        while(opModeIsActive()){
            left_y = gamepad1.left_stick_y;
            right_y = gamepad1.right_stick_y;
            left_x = gamepad1.left_stick_x;
            right_x = gamepad1.right_stick_x;

            if (Math.abs(left_y)<0.1){
                left_y = 0;
            }
            if (Math.abs(right_y)<0.1){
                right_y = 0;
            }
            if (Math.abs(left_x)<0.1){
                left_x = 0;
            }
            if (Math.abs(right_x)<0.1){
                right_x = 0;
            }

            if (gamepad1.a) {
                claw.closeBottom();
                claw.closeTop();
            }
            if (gamepad1.b) {
                claw.openBottom();
                claw.openTop();
            }

            if (gamepad1.x) {
                shoulder.setPosition(RobotConstants.shoulder_dropOffPos);
            }
            if (gamepad1.y) {
                shoulder.setPosition(0);
            }

            //if (gamepad1.x) {
            //    //arm.setPosition(RobotConstants.arm_maxPos);
            //}
            //if (gamepad1.y) {
                //driveTrain.testMotors(5000,.5);
                //arm.setPosition(RobotConstants.arm_minPos);
            //}
            if (gamepad1.dpad_up) {
                wposition += .01;
            }
            if (gamepad1.dpad_down) {
                wposition -= 0.01;
            }

            //shoulder.moveManual(right_y);
            double correction = 0;
            correction = shoulder.update();
            //arm.update();

            arm.moveManual(left_y);
            //lift.moveManual(left_x);

            wrist.moveWrist(wposition);

            //driveTrain.drive(left_x,left_y,0);

            //telemetry.addData("figureoutshort", driveTrain.figureOutWhatIsShorter());
            //telemetry.addData("turning info",driveTrain.getWhatHeadingDo());
            telemetry.addData("arm Position", arm.getEncoderValue() );
            telemetry.addData("arm Target",arm.getTarget());
            telemetry.addData("shoulder Position", shoulder.getEncoderValue());
            telemetry.addData("shoulder Target", shoulder.getTarget());
            //telemetry.addData("lift position", lift.getEncoderValue());
            telemetry.addData("wrist pos", wposition);
            telemetry.addData("shoulder correction", correction);
            telemetry.addData("stick", left_y);
            //telemetry.addData("perp wheel", driveTrain.perpReturn());
            //telemetry.addData("inches perp", driveTrain.perpReturnInches());
            //telemetry.addData("par wheel", driveTrain.parReturn());
            //telemetry.addData("inches par", driveTrain.parReturnInches());

            telemetry.update();
        }
    }
}
