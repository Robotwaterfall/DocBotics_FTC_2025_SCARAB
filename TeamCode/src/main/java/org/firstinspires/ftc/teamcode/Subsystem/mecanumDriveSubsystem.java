package org.firstinspires.ftc.teamcode.Subsystem;

import static org.firstinspires.ftc.teamcode.Constants.countsPerRevolution;
import static org.firstinspires.ftc.teamcode.Constants.gearRatio;
import static org.firstinspires.ftc.teamcode.Constants.kPx;
import static org.firstinspires.ftc.teamcode.Constants.wheelDiameter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mecanumDriveSubsystem extends SubsystemBase {

    private final DcMotor m_Fl, m_Fr, m_Rl, m_Rr;

    // Store last joystick values for telemetry
    private double fwdPower, strPower, rotPower;

    double motorFwd = 0;

    double DistancePerTick = (Math.PI * wheelDiameter) / (countsPerRevolution * gearRatio);

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public mecanumDriveSubsystem(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, HardwareMap hardwareMap) {
        m_Fl = frontLeft;
        m_Fr = frontRight;
        m_Rl = backLeft;
        m_Rr = backRight;

        // Set motor directions (typical mecanum setup)
        m_Fl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_Fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m_Rl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_Rr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetDriveEncoders() {
        m_Fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_Fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_Rl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_Rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_Fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_Fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_Rl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_Rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        double fl = forward + strafe + rotation;
        double fr = forward - strafe + rotation;
        double bl = forward - strafe - rotation;
        double br = forward + strafe - rotation;

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
    public double getYinches(){
        double Fl = m_Fl.getCurrentPosition();
        double Fr = m_Fr.getCurrentPosition();
        double Bl = m_Rl.getCurrentPosition();
        double Br = m_Rr.getCurrentPosition();

        double forwardError = (Fl - Fr - Bl + Br) / 4.0;

        double forwardMovement = forwardError * DistancePerTick;


        return forwardMovement;

    }

    public double getXinches(){
        double Fl = m_Fl.getCurrentPosition();
        double Fr = m_Fr.getCurrentPosition();
        double Bl = m_Rl.getCurrentPosition();
        double Br = m_Rr.getCurrentPosition();

        double lateralError = (Fl + Fr + Bl + Br) / 4.0;

        double lateralMovement = lateralError * DistancePerTick;


        return lateralMovement;

    }

    public void xCordinate(double desiredX) {
        double error = getXinches() - desiredX;

        motorFwd = error * kPx;

        motorFwd = Math.min(Math.max(motorFwd, -1.0), 1.0);

        drive(motorFwd,0,0);
    }




    @Override
    public void periodic() {
        // Send telemetry to FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Forward", fwdPower);
        packet.put("Strafe", strPower);
        packet.put("Rotation", rotPower);
        packet.put("FL Power", m_Fl.getPower());
        packet.put("FR Power", m_Fr.getPower());
        packet.put("BL Power", m_Rl.getPower());
        packet.put("BR Power", m_Rr.getPower());
        packet.put("distanceSidewaysInches", getXinches());
        packet.put("distanceForwardInches", getYinches());
        packet.put("distanceFromSetpoint", motorFwd);
        dashboard.sendTelemetryPacket(packet);
    }
}