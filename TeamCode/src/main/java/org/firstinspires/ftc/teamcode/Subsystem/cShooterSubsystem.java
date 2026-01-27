package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class cShooterSubsystem extends SubsystemBase {

    DcMotorEx shooterMotor1;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public cShooterSubsystem(DcMotorEx m_shooterMotor1){
        this.shooterMotor1 = m_shooterMotor1;

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setShooterMotorPower(double motorPower){
        shooterMotor1.setPower(motorPower);

    }

    public double getMotorPower(){
        return shooterMotor1.getPower();
    }

    public DcMotorEx getShooterMotor1(){return shooterMotor1;}

    @Override
    public void periodic() {
        TelemetryPacket shooterPacket = new TelemetryPacket();
        shooterPacket.put("shooterMotorPower: ", getMotorPower());
        dashboard.sendTelemetryPacket(shooterPacket);
    }
}
