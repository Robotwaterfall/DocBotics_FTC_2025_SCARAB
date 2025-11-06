package org.firstinspires.ftc.teamcode.Subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class mecanumDriveSubsystem extends SubsystemBase {

    public final DcMotor m_Fl, m_Fr, m_Rl, m_Rr;

    // Store last joystick values for telemetry
    private double fwdPower, strPower, rotPower;

    private IMU imu;
    IMU.Parameters myIMUParameters;


    public mecanumDriveSubsystem(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, IMU imu, HardwareMap hardwareMap) {
        m_Fl = frontLeft;
        m_Fr = frontRight;
        m_Rl = backLeft;
        m_Rr = backRight;
        imu = imu;

        // Set motor directions (typical mecanum setup)
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
    }

    /**
     * Drive method for teleop.
     * @param forward forward/backward input (-1 to 1)
     * @param strafe left/right input (-1 to 1)
     * @param rotation rotation input (-1 to 1)
     */
    public void drive(double forward, double strafe, double rotation) {
        // Apply deadzone
        forward = applyDeadzone(forward, 0.05);
        strafe  = applyDeadzone(strafe, 0.05);
        rotation = applyDeadzone(rotation, 0.05);

        fwdPower = forward;
        strPower = strafe;
        rotPower = rotation;


        // Mecanum kinematics
        double fl = forward - strafe - rotation;
        double fr = forward + strafe - rotation;
        double bl = forward + strafe + rotation;
        double br = forward - strafe + rotation;

        // Normalize motor powers
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Set motor powers
        m_Fl.setPower(fl);
        m_Fr.setPower(fr);
        m_Rl.setPower(bl);
        m_Rr.setPower(br);
    }

    // Deadzone helper
    private double applyDeadzone(double value, double threshold) {
        return Math.abs(value) > threshold ? value : 0;
    }

    public void resetHeading(){
        imu.resetYaw();
    }

    public double getCurrentHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public DcMotor getFl(){
        return m_Fl;
    }
    public DcMotor getFr(){
        return m_Fr;
    }
    public DcMotor getRl(){
        return m_Rl;
    }
    public DcMotor getRr(){
        return m_Rr;
    }

    public double getFwdPower(){
        return fwdPower;
    }

    public double getStrPower(){
        return strPower;
    }

    public double getRotPower(){
        return rotPower;
    }

    public void stopandresetEncoder(){
        getFl().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getFr().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getRl().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getRr().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder(){
        getFl().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        getFr().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        getRl().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        getRr().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEndcoder(){
        getFl().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getFr().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getRl().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        getRr().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder(){
        stopandresetEncoder();
        runUsingEncoder();
    }

}