package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class waitCommand extends CommandBase {

    private final int m_waitTimeSec;
    private final ElapsedTime runTime = new ElapsedTime();

    public waitCommand(int waitTimeSec){
        m_waitTimeSec = waitTimeSec;

    }
    @Override
    public void initialize() {

        runTime.reset();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {

        return m_waitTimeSec <= runTime.seconds();

    }
}
