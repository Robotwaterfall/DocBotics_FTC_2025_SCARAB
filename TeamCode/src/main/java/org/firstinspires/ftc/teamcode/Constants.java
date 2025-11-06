package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // =======PID Gains=======
    public static double kPRotation            = 0.02;
    public static double kPForward             = 0.04;
    public static double kPLateral             = 0.04;

    // =======Tolerance and Deadband=======
    public static double tolerance             = 0.05; //offset of the rotation and forwarding
    public static double deadband              = 1.5; //degrees in which we stop rotating

    // =======Target Setpoints=======
    public static double targetDistanceInches     = 2;

    public static double catapult_up_power        = -1.0;
    public static double catapult_down_power      = 1.0;

    // =======Motor Configs==========

    public static final int MOTOR_TICK_COUNTS  = 1120;
    public static final double wheelDiameter = 3.0; //diameter of the wheel
    public static final int gearRatio = 21;
    public static final int countsPerRevolution = 28;

    public static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // REV ultraplanetary motor
    public static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // External Gearing.
    public static final double     WHEEL_DIAMETER_INCHES   = 3 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                             (WHEEL_DIAMETER_INCHES * Math.PI);


    public static final class AutoConstants{
        public static boolean isArmJointLimiterOff;
        public static final double turnkP = 0.01;
        public static final double turnkI = 0;
        public static final double turnkD = 0;
        public static final double turnkF = 0;

        public static final double movekP = 0.06;
        public static final double movekI = 0;
        public static final double movekD = 0.01;
        public static final double movekF = 0;
    }
    public static final class encoderAutoConstants{
        public static final double     DRIVE_SPEED             = 0.25;
        public static final double     TURN_SPEED              = 0.5;

        public static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // REV ultraplanetary motor
        public static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // External Gearing.
        public static final double     WHEEL_DIAMETER_INCHES   = 3 ;     // For figuring circumference
        public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);

    }


    private Constants() { }

}
