package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.kPRotation;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

import java.util.function.Supplier;

public class teleOpMecanumDriveCommand extends CommandBase {


    private final mecanumDriveSubsystem driveSub;
    private final limelightSubsystem llsub;
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rSupplier;

    public teleOpMecanumDriveCommand(
            mecanumDriveSubsystem driveSub,
            limelightSubsystem llsub,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> rSupplier
            ) {

        this.driveSub = driveSub;
        this.llsub = llsub;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;
        addRequirements(driveSub);
    }

    @Override
    public void execute() {
            //Normal teleoperated drive
            double forward = ySupplier.get();
            double strafe  = xSupplier.get();
            double rotation = rSupplier.get();

            driveSub.drive(forward, strafe, rotation);

    }
}