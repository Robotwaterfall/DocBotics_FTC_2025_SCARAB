package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

public class intakeSubsystem extends SubsystemBase {

    DcMotor m_intakeMotor;
    boolean intakeRunning = false;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public intakeSubsystem(DcMotor intakeMotor){
        m_intakeMotor = intakeMotor;

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); //TODO: check intake motor direction
    }

    public void setM_intakeMotorPower(double power){
        intakeRunning = true;
        m_intakeMotor.setPower(power);
    }

    public boolean isIntakeRunning(){
        return intakeRunning;
    }

    @Override
    public void periodic() {
        TelemetryPacket intakePacket = new TelemetryPacket();
        intakePacket.put("intakeRunning: ", isIntakeRunning());
        intakePacket.put("intakePower", m_intakeMotor.getPower());
        dashboard.sendTelemetryPacket(intakePacket);

    }
}
