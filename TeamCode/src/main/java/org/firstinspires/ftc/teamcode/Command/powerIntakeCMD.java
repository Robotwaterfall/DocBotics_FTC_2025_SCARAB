package org.firstinspires.ftc.teamcode.Command;

import static org.firstinspires.ftc.teamcode.Constants.intake_POWER;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.intakeSubsystem;

public class powerIntakeCMD extends CommandBase {
    private final intakeSubsystem intakeSub;
    private final double intakePower;
    public powerIntakeCMD(intakeSubsystem intakeSub, double intakePower){
        this.intakeSub = intakeSub;
        this.intakePower = intakePower;
        addRequirements(intakeSub);

    }

    @Override
    public void initialize() {
        intakeSub.setM_intakeMotorPower(0);
    }

    @Override
    public void execute() {
       intakeSub.setM_intakeMotorPower(intakePower);
    }

    @Override
    public void end(boolean interrupted) {
        //when cmd is done stop intake motor
       intakeSub.setM_intakeMotorPower(0);
    }
}
