package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //Rev motor constants
        public static final int TICKS_PER_REV = 28;
        public static final int MAX_RPM = 6000;

    //shoulder constants
        public static final double shoulder_GEAR_RATIO = (108.0/16.0)*3*3*4;
        public static final double shoulder_kP = 0.008;
        public static final double shoulder_kI = 0;
        public static final double shoulder_kD = 0;
        public static final double shoulder_kG = -0.1;
        public static final int shoulder_maxVel = 500000;
        public static final int shoulder_maxAccel = 4000;
        public static final int shoulder_maxJerk = 20000;
        public static int shoulder_dropOffPos = -2000;
        public static int shoulder_stowPos = -1000;
        public static int shoulder_pickupPos = -500;


    //telescope constants
        public static double telescope_GEAR_RATIO = 4*4*4;
        public static double telescope_kP = 0.01;
        public static double telescope_kI = 0;
        public static double telescope_kD = 0;
        public static double telescope_kG = 0.025;
        public static int telescope_maxVel = 5000;
        public static int telescope_maxAccel = 5000;
        public static int telescope_maxJerk = 5000;
        public static int telescope_maxPos = -3000;
        public static int telescope_minPos = -600;
        public static int telescope_stowPos = -1000;
        public static int telescope_pickupPos = -1000;
        public static int telescope_dropOffLow = -1000;
        public static int telescope_dropOffMid = -1000;
        public static int telescope_dropOffHigh = -1000;

    //claw constants
        public static double claw_openBottom = 0.4;
        public static double claw_closeBottom = 0.63;
        public static double claw_openTop = 0.38;
        public static double claw_closeTop = 0.6;

    //Wrist constants
        public static double wrist_stowPos = 1.0;
        public static double wrist_pickupPos = 0.5;
        public static double wrist_dropOffPos = 0.5;

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
}
