package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.intake_POWER;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystem.intakeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class teleOpIntakeCommand extends CommandBase {
    intakeSubsystem intakeSub;
    DoubleSupplier leftTriggerSupplier;



    public teleOpIntakeCommand(intakeSubsystem intakeSub, DoubleSupplier leftTriggerSupplier ){
        this.intakeSub = intakeSub;

        this.leftTriggerSupplier = leftTriggerSupplier;


        addRequirements(intakeSub);
    }

    @Override
    public void initialize(){
            intakeSub.setM_intakeMotorPower(0);

    }

    @Override
    public void execute(){


        double leftT = leftTriggerSupplier.getAsDouble();

        intakeSub.setM_intakeMotorPower(leftT);


    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.setM_intakeMotorPower(0);
    }
}
