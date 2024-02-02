package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

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
    private Encoder perpEncoder;
    private Encoder parEncoder;
    NavxMicroNavigationSensor navX;
    IntegratingGyroscope gyro;
    private double headingToMaintain = 0;
    private String whatHeadingDo;

    public DriveSystemV2(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR, NavxMicroNavigationSensor n, Encoder parallel, Encoder perpindicular) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;
        this.parEncoder = parallel;
        this.perpEncoder = perpindicular;
        this.navX = n;

        this.frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.gyro = (IntegratingGyroscope)navX;
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
        return -result;
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

