package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.intakeSubsystem;

public class powerIntakeCMD extends CommandBase {

    private final intakeSubsystem intakeSub;
    private  final double power;

    public powerIntakeCMD(intakeSubsystem intakeSub, double power){

        this.intakeSub = intakeSub;
        this.power = power;

    }
    @Override
    public void initialize(){
        intakeSub.setM_intakeMotorPower(0);

    }

    @Override
    public void execute(){



        intakeSub.setM_intakeMotorPower(power);


    }

    @Override
    public void end(boolean interrupted) {
        intakeSub.setM_intakeMotorPower(0);
    }
}




