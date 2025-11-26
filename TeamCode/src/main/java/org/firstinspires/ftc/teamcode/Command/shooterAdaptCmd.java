package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.max_Shooter_Power;
import static org.firstinspires.ftc.teamcode.Constants.min_Shooter_Power;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystem.cShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;

public class shooterAdaptCmd extends CommandBase {

    limelightSubsystem llSub;
    cShooterSubsystem shooterSub;

    public shooterAdaptCmd(limelightSubsystem llSub, cShooterSubsystem shooterSub){
        this.llSub = llSub;
        this.shooterSub = shooterSub;
        addRequirements(shooterSub);
    }

    @Override
    public void initialize() {
        shooterSub.setShooterMotorPower(0);
    }

    @Override
    public void execute() {

        double ty = llSub.getTy();


    // 1) Clamp ty to a sane range (numbers you see in practice)
        double tyMin = -10;   // tune from real data
        double tyMax =  10;

        if (ty < tyMin) ty = tyMin;
        if (ty > tyMax) ty = tyMax;

    // 2) Normalize ty -> 0..1  (or 0..100)
        double normalized = (ty - tyMin) / (tyMax - tyMin); // 0..1
        double shooterPercent = normalized * 100.0;         // 0..100

        double minPower = min_Shooter_Power;
        double maxPower = max_Shooter_Power;

        double shooterPower = minPower + (shooterPercent / 100.0) * (maxPower - minPower);

        // clamp again just in case
        if (shooterPower < minPower) shooterPower = minPower;
        if (shooterPower > maxPower) shooterPower = maxPower;

        shooterSub.setShooterMotorPower(shooterPower);


    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.setShooterMotorPower(0);
    }
}
