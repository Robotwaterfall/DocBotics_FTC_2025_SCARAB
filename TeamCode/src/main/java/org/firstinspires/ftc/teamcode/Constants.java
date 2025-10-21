package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    public static double  soos_Linear_scaler = 1.1975;
    public static double  soos_Angular_scaler = 0.98622;

    //drivetrain feedforward constants

    //measured at 13.4V
    public static double  drivetrain_ks = 0.09;
    //measured at 13.2V
    public static double  drivetrain_kv = 0.00945;
    //measured at 13.2
    public static double  drivetrain_ka = 0.00124;


    //drivetrain Trapezoidal drive constants
    public static double drivetrain_maxAccel = 10.0; //inches/sec^2
    public static double drivetrain_maxVelocity = 15.0; //inches/sec
    public static double drivetrain_targetDistance = 20.0; // Measured in inches



    private Constants() { }

}
