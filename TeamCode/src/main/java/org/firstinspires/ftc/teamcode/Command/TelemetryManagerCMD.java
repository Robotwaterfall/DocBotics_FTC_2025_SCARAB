package org.firstinspires.ftc.teamcode.Command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystem.TelemetryManagerSubsystem;

public class TelemetryManagerCMD extends CommandBase {
    TelemetryManagerSubsystem telemetryManager;

    public TelemetryManagerCMD(TelemetryManagerSubsystem telemetryManager){
        this.telemetryManager = telemetryManager;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        telemetryManager.runTelemetry();
    }
}
