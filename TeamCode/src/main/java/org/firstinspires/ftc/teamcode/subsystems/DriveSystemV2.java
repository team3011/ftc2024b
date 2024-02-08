package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Locale;

public class DriveSystemV2 {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    NavxMicroNavigationSensor navX;
    IntegratingGyroscope gyro;
    private double headingToMaintain = 0;
    private String whatHeadingDo;
    private int xTarget;
    private int yTarget;
    private double x_correction;
    private double y_correction;
    private PIDCoefficients x_coeffs;
    private PIDFController x_controller;
    private MotionProfile x_profile;
    private ElapsedTime x_timer = new ElapsedTime();
    private PIDCoefficients y_coeffs;
    private PIDFController y_controller;
    private MotionProfile y_profile;
    private ElapsedTime y_timer = new ElapsedTime();

    public DriveSystemV2(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR, NavxMicroNavigationSensor n) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;
        this.navX = n;

        this.frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.gyro = (IntegratingGyroscope)this.navX;

        this.x_coeffs = new PIDCoefficients(RobotConstants.x_kP,RobotConstants.x_kI,RobotConstants.x_kD);
        this.x_controller = new PIDFController(this.x_coeffs,0,0,0,(x,y)->RobotConstants.x_kG);
        this.x_controller.setOutputBounds(-1,1);

        this.x_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0,0,0),
                new MotionState(0,0,0),
                RobotConstants.x_maxVel,
                RobotConstants.x_maxAccel,
                RobotConstants.x_maxJerk
        );
        this.y_coeffs = new PIDCoefficients(RobotConstants.y_kP,RobotConstants.y_kI,RobotConstants.y_kD);
        this.y_controller = new PIDFController(this.y_coeffs,0,0,0,(x,y)->RobotConstants.y_kG);
        this.y_controller.setOutputBounds(-1,1);

        this.y_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0,0,0),
                new MotionState(0,0,0),
                RobotConstants.y_maxVel,
                RobotConstants.y_maxAccel,
                RobotConstants.y_maxJerk
        );

        this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getxTarget(){
        return this.xTarget;
    }
    public int getyTarget(){
        return this.yTarget;
    }
    public double getX_correction(){
        return x_correction;
    }
    public double getY_correction(){
        return y_correction;
    }

    public void setTarget(int mmX, int mmY){
        this.xTarget = mmX;
        this.yTarget = mmY;

        this.x_coeffs = new PIDCoefficients(RobotConstants.x_kP,RobotConstants.x_kI,RobotConstants.x_kD);
        this.x_controller = new PIDFController(this.x_coeffs,0,0,0,(x,y)->RobotConstants.x_kG);
        this.x_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getXDistance(),0,0),
                new MotionState(mmX,0,0),
                RobotConstants.x_maxVel,
                RobotConstants.x_maxAccel,
                RobotConstants.x_maxJerk
        );
        this.x_timer.reset();

        this.y_coeffs = new PIDCoefficients(RobotConstants.y_kP,RobotConstants.y_kI,RobotConstants.y_kD);
        this.y_controller = new PIDFController(this.y_coeffs,0,0,0,(x,y)->RobotConstants.y_kG);
        this.y_profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getYDistance(),0,0),
                new MotionState(mmY,0,0),
                RobotConstants.y_maxVel,
                RobotConstants.y_maxAccel,
                RobotConstants.y_maxJerk
        );
        this.y_timer.reset();
    }

    private int convertToMM(int input){
        return (int) (input*0.15073*RobotConstants.drive_fudge/2);
    }

    public int getXDistance(){
        return convertToMM(-this.backLeft.getCurrentPosition());
    }

    public int getYDistance(){
        return convertToMM(-this.backRight.getCurrentPosition());
    }
    public void update(){
        double x_correction = 0;
        double y_correction = 0;
        MotionState stateX = this.x_profile.get(this.x_timer.seconds());
        this.x_controller.setTargetPosition(stateX.getX());
        this.x_controller.setTargetVelocity(stateX.getV());
        this.x_controller.setTargetAcceleration(stateX.getA());
        x_correction = x_controller.update(getXDistance());
        MotionState stateY = this.y_profile.get(this.y_timer.seconds());
        this.y_controller.setTargetPosition(stateY.getX());
        this.y_controller.setTargetVelocity(stateY.getV());
        this.y_controller.setTargetAcceleration(stateY.getA());
        y_correction = y_controller.update(getYDistance());
        drive(y_correction,x_correction,0);

    }

    public void testMotors(int delay, double power) throws InterruptedException {
        this.frontLeft.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(power);
        Thread.sleep(delay);
        this.frontRight.setPower(0);
        this.backLeft.setPower(power);
        Thread.sleep(delay);
        this.backLeft.setPower(0);
        this.backRight.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
        this.backLeft.setPower(0);
        this.backRight.setPower(0);

    }

    /** main thread for this class, commands the motors to do required movements
     *
     * @param leftStickX commands robot strafing movements
     * @param leftStickY commands robot forward and backward movements
     * @param rightStickX commands robot's rotation
     */
    public void drive(double leftStickX, double leftStickY, double rightStickX) {
        //x=y
        //y=x
        //-,-,+ f goes b, l goes r,rot r goes left
        //+,+,- f goes f, l goes l,rot r goes right


        double x = -leftStickY * RobotConstants.MULTIPLIER;
        double y = -leftStickX * RobotConstants.MULTIPLIER; // Counteract imperfect strafing
        double rx = -rightStickX * 0.5 * RobotConstants.MULTIPLIER; //what way we want to rotate

        Orientation angles = this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double robotHeading = Math.toRadians(angles.firstAngle);

        if(rx == 0){ //this means that we're trying to maintain our current heading

            //prevents the motors from working when they realistically cant
            boolean isWithinAngularTolerance =
                    Math.abs(this.figureOutWhatIsShorter()) < RobotConstants.ANGULAR_TOLERANCE;

            //we turn if we're not within a tolerance
            if(!isWithinAngularTolerance){
                rx = this.figureOutWhatIsShorter(); //retrieves direction, magnitude overwritten
                rx = this.setToMinimumTurningSpeed(rx); //overwrites magnitude to minimum speed
            }

            this.whatHeadingDo =
                    String.format(Locale.getDefault(),"Maintaining Desired Heading of %.2f",
                            Math.toDegrees(headingToMaintain));
        }else{
            this.whatHeadingDo = "Turning";
            //we're going to maintain our new heading once we've stopped turning.
            //not before we've turned
            this.headingToMaintain = robotHeading;
        }
        //triangle """magic"""
        double rotX = x * Math.cos(robotHeading) - y * Math.sin(robotHeading);
        double rotY = x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX - rx)  / denominator;
        double frontRightPower = (rotY - rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX - rx)   / denominator;
        double backRightPower = (rotY + rotX + rx)  / denominator;

        this.frontLeft.setPower(frontLeftPower);
        this.frontRight.setPower(frontRightPower);
        this.backLeft.setPower(backLeftPower);
        this.backRight.setPower(backRightPower);


    }

    /** Determines what direction would be shorter to turn in when trying to maintain our current
     *  heading.
     * @return the shorter difference in heading
     */
    public double figureOutWhatIsShorter() {
        double result;
        Orientation angles = this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double reading = Math.toRadians(angles.firstAngle);
        double oppositeButEqualReading;

        if (reading > 0) {
            oppositeButEqualReading = reading - 2 * Math.PI;
        } else {
            oppositeButEqualReading = reading + 2 * Math.PI;
        }

        double normalReadingDifference = Math.abs(this.headingToMaintain - reading);
        double oppositeReadingDifference = Math.abs(this.headingToMaintain - oppositeButEqualReading);

        boolean isOppositeReadingShorter =
                normalReadingDifference > oppositeReadingDifference;

        if (isOppositeReadingShorter) {
            result = this.headingToMaintain - oppositeButEqualReading;
        } else {
            result = this.headingToMaintain - reading;
        }
        return result;
    }

    /** changes our current turning speed to a turning speed that allows us to rotate
     *
     * @param rx our current turning speed
     * @return modified turning speed
     */
    private double setToMinimumTurningSpeed(double rx){

        if(Math.abs(rx) < RobotConstants.MINIMUM_TURNING_SPEED) {
            if (rx < 0) {
                return -RobotConstants.MINIMUM_TURNING_SPEED;
            } else {
                return RobotConstants.MINIMUM_TURNING_SPEED;
            }
        }else{
            return rx;
        }
    }

    public String getWhatHeadingDo(){
        return this.whatHeadingDo;
    }
}

