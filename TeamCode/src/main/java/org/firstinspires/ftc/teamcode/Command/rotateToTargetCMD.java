package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

public class rotateToTargetCMD extends CommandBase {
    private final mecanumDriveSubsystem drive;
    private final limelightSubsystem llSub;
    private final double kPRotation = Constants.kPRotation; //simple proportional gain
    private final double deadband = Constants.deadband; //degrees in which we stop
    public rotateToTargetCMD(mecanumDriveSubsystem drive, limelightSubsystem llSub){
        this.drive = drive;
        this.llSub = llSub;
        addRequirements(drive);
    }

    @Override
    public void execute(){
        if(llSub.hasTarget()) {
            double error = llSub.getTx(); //horizontal offset in degrees
            double rotPower = error * kPRotation; //negative to correct direction
            //clipped power to [-0.4, 0.4] for safety
            rotPower = Math.max(Math.min(rotPower, 0.4), -0.4);

            drive.drive(0, 0, rotPower); //rot only
        } else {
            drive.drive(0,0,0); //stop if not target is found
        }
    }

    @Override
    public boolean isFinished(){
        return llSub.hasTarget() && Math.abs(llSub.getTx()) < deadband;
    }

    @Override
    public void end(boolean interrupted){
        drive.drive(0,0,0);   //stop motors
    }
}
