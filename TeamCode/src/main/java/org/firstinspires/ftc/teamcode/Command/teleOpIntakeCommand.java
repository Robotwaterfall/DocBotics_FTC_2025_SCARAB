package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.intake_POWER;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.lynx.Supplier;

import org.firstinspires.ftc.teamcode.Subsystem.intakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class teleOpIntakeCommand extends CommandBase {
    intakeSubsystem intakeSub;
    DoubleSupplier rightTriggerSupplier;
    DoubleSupplier leftTriggerSupplier;

    public teleOpIntakeCommand(intakeSubsystem intakeSub, DoubleSupplier rightTriggerSupplier, DoubleSupplier leftTriggerSupplier ){
        this.intakeSub = intakeSub;
        this.rightTriggerSupplier = rightTriggerSupplier;
        this.leftTriggerSupplier = leftTriggerSupplier;
        addRequirements(intakeSub);
    }

    @Override
    public void initialize(){
            intakeSub.setM_intakeMotorPower(0);

    }

    @Override
    public void execute(){

        double rightT = rightTriggerSupplier.getAsDouble();
        double leftT = leftTriggerSupplier.getAsDouble();

        intakeSub.setM_intakeMotorPower(rightT);

        intakeSub.setM_intakeMotorPower(-leftT);

    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.setM_intakeMotorPower(0);
    }
}
