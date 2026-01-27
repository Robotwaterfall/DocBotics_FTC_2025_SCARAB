package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.max_Shooter_Power;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystem.cShooterSubsystem;

public class findTPSCmd extends CommandBase {

    public final cShooterSubsystem cShooter;

    public final DcMotorEx shooterMotor;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public findTPSCmd(cShooterSubsystem cShooter){
        this.cShooter = cShooter;
        this.shooterMotor = cShooter.getShooterMotor1();
        addRequirements(cShooter);

    }

    @Override
    public void initialize() {
        shooterMotor.setPower(0);

    }

    @Override
    public void execute() {

        shooterMotor.setVelocity(max_Shooter_Power);


        TelemetryPacket tpsfinder = new TelemetryPacket();
        tpsfinder.put("TPS", shooterMotor.getVelocity());
        dashboard.sendTelemetryPacket(tpsfinder);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
