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
        public static int shoulder_maxVel = 50000;
        public static int shoulder_maxAccel = 3000;
        public static int shoulder_maxJerk = 20000;
        public static final int shoulder_dropOffPos = -2125;
        public static final int shoulder_stowPos = 0;
        public static final int shoulder_pickupPos = 0;


    //telescope constants
        public static final double telescope_GEAR_RATIO = 4*4*4;
        public static final double telescope_kP = 0.01;
        public static final double telescope_kI = 0;
        public static final double telescope_kD = 0;
        public static final double telescope_kG = 0.025;
        public static int telescope_maxVel = 10000;
        public static int telescope_maxAccel = 10000;
        public static int telescope_maxJerk = 2000;
        public static final int telescope_stowPos = -300;
        public static final int telescope_pickupPos = -1500;
        public static final int telescope_dropOffHigh = -3000;

    //claw constants
        public static final double claw_openBottom = 0.4;
        public static final double claw_closeBottom = 0.64;
        public static final double claw_openTop = 0.38;
        public static final double claw_closeTop = 0.6;
        public static final double claw_partialTop = 0.44;
        public static final double claw_partialBottom = 0.42;

        public static final int claw_pickupPause = 1000;
        public static final int claw_dropTopPause = 1000;
        public static final int claw_dropBottomPause = 1000;


    //Wrist constants
        public static double wrist_stowPos = 0.60;
        public static double wrist_pickupPos = 0.1;
        public static double wrist_dropOffPos = 0.65;
        public static final int wrist_stowTime = 1000;
        public static final int wrist_pickupTime = 1000;
        public static final int wrist_dropOffTime = 1000;
        public static final double wrist_temp = 0.5;

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
        public static final double x_kP = 0.009;
        public static final double x_kI = 0;
        public static final double x_kD = 0.0001;
        public static final double x_kG = 0;
        public static final int x_maxVel = 5000;
        public static final int x_maxAccel = 500;
        public static final int x_maxJerk = 500;
        public static final double y_kP = 0.006;
        public static final double y_kI = 0;
        public static final double y_kD = 0.001;
        public static final double y_kG = 0;
        public static final int y_maxVel = 5000;
        public static final int y_maxAccel = 500;
        public static final int y_maxJerk = 500;
        public static final int yTarget = 0;
        public static final int xTarget = 500;
        public static final double drive_fudge = 1;
        public static final double speedB = 10;
        public static final double speedE = 10;

}
