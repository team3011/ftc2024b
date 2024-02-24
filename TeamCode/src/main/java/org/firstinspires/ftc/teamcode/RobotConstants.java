package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static double ringPower = 1;
    public static float cameraConf = .4f;
    public static int yForPickup = -1550;
    public static int xForPickup = -1300;
    //shoulder constants
        public static double shoulder_kP = 0.015;
        public static double shoulder_kI = 0;
        public static double shoulder_kD = 0;
        public static double shoulder_kG = -0.25;
        public static int shoulder_maxVel = 20000;
        public static int shoulder_maxAccel = 2500;
        public static int shoulder_maxJerk = 1200;
        public static int shoulder_dropOffPos = -2100;
        public static final int shoulder_stowPos = 0;
        public static final int shoulder_pickupPos = 0;
        public static int shoulder_stack1 = -48;
        public static int shoulder_stack2 = -67;
        //public static int shoulder_stack3 = -94;
        public static int shoulder_stack3 = -110;


    //telescope constants
        public static final double telescope_kP = 0.01;
        public static final double telescope_kI = 0;
        public static final double telescope_kD = 0;
        public static final double telescope_kG = 0.025;
        public static final int telescope_maxVel = 10000;
        public static final int telescope_maxAccel = 10000;
        public static final int telescope_maxJerk = 2000;
        public static int telescope_stowPos = -500;
        public static int telescope_pickupPos = -1000;
        public static final int telescope_dropOffHigh = -3700;

    //claw constants
        public static double claw_openBottom = 0.1;
        public static double claw_closeBottom = 0.42;
        public static double claw_openTop = 0.08;
        public static double claw_closeTop = 0.27;
        public static double claw_partialTop = 0.15;
        public static double claw_partialBottom = 0.3;

        public static final int claw_pickupPause = 250;
        public static final int claw_dropTopPause = 400;
        public static final int claw_dropBottomPause = 600;


    //Wrist constants
        public static final double wrist_stowPos = 0.63;
        public static final double wrist_pickupPos = 0.06;
        public static final double wrist_dropOffPos = 0.65;
        public static final int wrist_stowTime = 1000;
        public static final int wrist_pickupTime = 1000;
        public static final int wrist_dropOffTime = 1000;
        public static double wrist_temp = 0.60;

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
        public static int y_maxVel = 7000;
        public static int y_maxAccel = 700;
        public static int y_maxJerk = 700;
        public static final int yTarget = 0;
        public static final int xTarget = 500;
        public static final double drive_fudge = 1;
        public static final double speedB = 10;
        public static final double speedE = 10;

}
