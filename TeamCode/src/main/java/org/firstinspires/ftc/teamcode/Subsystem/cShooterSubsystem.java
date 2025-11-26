package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class cShooterSubsystem extends SubsystemBase {

    DcMotor shooterMotor;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public cShooterSubsystem(DcMotor m_shooterMotor){
        this.shooterMotor = m_shooterMotor;

    }

    public void setShooterMotorPower(double motorPower){
        shooterMotor.setPower(motorPower);
    }
}
