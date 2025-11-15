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
        //turn off power to intake motors.
        intakeSub.setM_intakeMotorPower(0);

    }

    @Override
    public void execute(){
        //set intake motors to desired speeds
        intakeSub.setM_intakeMotorPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        // When Command ends stop intake motors
        intakeSub.setM_intakeMotorPower(0);
    }
}




