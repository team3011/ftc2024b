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
        public static final int shoulder_dropOffPos = -2000;
        public static final int shoulder_stowPos = -150;
        public static final int shoulder_pickupPos = 0;


    //telescope constants
        public static final double telescope_GEAR_RATIO = 4*4*4;
        public static final double telescope_kP = 0.01;
        public static final double telescope_kI = 0;
        public static final double telescope_kD = 0;
        public static final double telescope_kG = 0.025;
        public static final int telescope_maxVel = 50000;
        public static final int telescope_maxAccel = 50000;
        public static final int telescope_maxJerk = 50000;
        public static final int telescope_stowPos = -300;
        public static final int telescope_pickupPos = -1500;
        public static final int telescope_dropOffHigh = -3000;

    //claw constants
        public static final double claw_openBottom = 0.4;
        public static final double claw_closeBottom = 0.64;
        public static final double claw_openTop = 0.38;
        public static final double claw_closeTop = 0.6;
        public static int claw_pickupPause = 1000;

    //Wrist constants
        public static double wrist_stowPos = 0.85;
        public static double wrist_pickupPos = 0.53;
        public static double wrist_dropOffPos = 0.45;
        public static int wrist_stowTime = 3000;
        public static int wrist_pickupTime = 3000;
        public static int wrist_dropOffTime = 3000;

    //lift constants
        public static final double lift_kP = 0.01;
        public static final double lift_kI = 0;
        public static final double lift_kD = 0;
        public static final double lift_kG = 0.025;
        public static final int lift_maxVel = 5000;
        public static final int lift_maxAccel = 500;
        public static final int lift_maxJerk = 500;
        public static final int lift_maxPos = -3000;
        public static final int lift_minPos = -600;

    //drive constants
        public static final float yawMax = 5;
        public static final float yawCheck = 2.5F;
        public static final double MULTIPLIER = 1;
        public static final double STICK_TOLERANCE = 0.1;
        public static final double MINIMUM_TURNING_SPEED = 0.05;
        public static final double ANGULAR_TOLERANCE = Math.toRadians(0.5);
}
