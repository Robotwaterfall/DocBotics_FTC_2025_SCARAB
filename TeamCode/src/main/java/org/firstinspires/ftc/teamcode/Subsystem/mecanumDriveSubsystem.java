package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class mecanumDriveSubsystem extends SubsystemBase {

    public final Motor m_Fl, m_Fr, m_Rl, m_Rr;
    public double fwdPower;
    public double strPower;
    public double rotPower;
    private final IMU imu;
    private final PIDController headingController;
    private double targetHeading;

    public mecanumDriveSubsystem(Motor front_left, Motor front_right,
                                 Motor back_left, Motor back_right,
                                 HardwareMap hardwareMap) {

        m_Fl = front_left;
        m_Fr = front_right;
        m_Rl = back_left;
        m_Rr = back_right;

        // Invert right side motors (FTC mecanum standard)
        m_Fr.setInverted(true);
        m_Rr.setInverted(true);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu.initialize(parameters);

        // Reset IMU so heading = 0 when robot faces forward
        imu.resetYaw();

        // PID controller for heading hold
        headingController = new PIDController(0.02, 0, 0.001);
        targetHeading = 0;
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * wraps an angle in radians to the range [-pi, pi]
     */
    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void drive(double forward, double strafe, double rotation,
                      boolean fieldCentric, boolean headingLock) {
        fwdPower = forward;
        strPower = strafe;
        rotPower = rotation;

        // Field-centric transform
        if (fieldCentric) {
            double heading = getHeading();
            double temp = forward * Math.cos(heading) + strafe * Math.sin(heading);
            strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
            forward = temp;
        }

        // Heading lock (only if driver isn't commanding rotation)
        if (headingLock && Math.abs(rotation) < 0.05) {
            double error = wrapAngle(targetHeading - getHeading());
            rotation = headingController.calculate(0, error);  // drive error to 0
        } else {
            targetHeading = getHeading();
        }

        // Mecanum kinematics
        double front_left  = forward + strafe + rotation;
        double front_right = forward - strafe - rotation;
        double back_left   = forward - strafe + rotation;
        double back_right  = forward + strafe - rotation;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(front_left),
                Math.max(Math.abs(front_right), Math.max(Math.abs(back_left), Math.abs(back_right)))));

        // Set motor powers
        m_Fl.set(front_left / max);
        m_Fr.set(front_right / max);
        m_Rl.set(back_left / max);
        m_Rr.set(back_right / max);
    }

    // Power getters
    public double getFwdPower() { return fwdPower; }
    public double getStrPower() { return strPower; }
    public double getRotPower() { return rotPower; }

    // Motor getters
    public Motor getM_Fl() { return m_Fl; }
    public Motor getM_Fr() { return m_Fr; }
    public Motor getM_Rl() { return m_Rl; }
    public Motor getM_Rr() { return m_Rr; }
}