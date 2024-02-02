package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //Rev motor constants
    public static final int TICKS_PER_REV = 28;
    public static final int MAX_RPM = 6000;

    //shoulder constants
    public static final double shoulder_GEAR_RATIO = (108.0/16.0)*3*3*4;
    public static double shoulder_kP = 0.008;
    public static double shoulder_kI = 0;
    public static double shoulder_kD = 0;
    public static double shoulder_kG = -0.1;
    public static int shoulder_maxVel = 500000;
    public static int shoulder_maxAccel = 4000;
    public static int shoulder_maxJerk = 20000;
    public static int shoulder_dropOffPos = -2000;

    //arm constants
    public static final double arm_GEAR_RATIO = 4*4*4;
    public static final double arm_kP = 0.01;
    public static final double arm_kI = 0;
    public static final double arm_kD = 0;
    public static final double arm_kG = 0.025;
    public static final int arm_maxVel = 5000;
    public static final int arm_maxAccel = 5000;
    public static final int arm_maxJerk = 5000;
    public static int arm_maxPos = -3000;
    public static int arm_minPos = -600;

    //Jullians claw constants
    public static double julliansClawOpenBottom = 0.4;
    public static double julliansClawCloseBottom = 0.63;
    public static double julliansClawOpenTop = 0.38;
    public static double julliansClawCloseTop = 0.6;



    //lift constants
    public static final double lift_kP = 0.01;
    public static final double lift_kI = 0;
    public static final double lift_kD = 0;
    public static final double lift_kG = 0.025;
    public static final int lift_maxVel = 5000;
    public static final int lift_maxAccel = 500;
    public static final int lift_maxJerk = 500;
    public static int lift_maxPos = -3000;
    public static int lift_minPos = -600;

    //drive constants
    public static final float yawMax = 5;
    public static final float yawCheck = 2.5F;
    public static double MULTIPLIER = 1;
    public static final double STICK_TOLERANCE = 0.1;
    public static final double MINIMUM_TURNING_SPEED = 0.05;
    public static final double ANGULAR_TOLERANCE = Math.toRadians(0.5);



    //public static int maxDifference = 200;
    //public static double kP = 0.01;
    //public static double kI = 0;
    //public static double kD = 0;
    //public static double kG = 0.025;

    //public static double Ticks_Per_Rev  = 28;
    //public static double GR = 60;
    //public static double TPR_CM = 2.2282;
    //public static int RPM = 6000;
    //public static int maxVel = (int)(RobotConstants.RPM * RobotConstants.Ticks_Per_Rev / RobotConstants.GR);
    //public static int maxAccel = (int)(5000);
    //public static int maxJerk = (int)(5000);
    //public static int max_Ticks_Raw = 200;
    //public static double Reduction = .95;
    //public static double WHEEL_RADIUS = 1.88976; // in
    //public static double TRACK_WIDTH = 16.34; // in

}
