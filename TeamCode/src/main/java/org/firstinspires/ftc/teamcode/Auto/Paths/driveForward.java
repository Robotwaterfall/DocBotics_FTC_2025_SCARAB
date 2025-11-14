package org.firstinspires.ftc.teamcode.Auto.Paths;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.MoveRobotEncoderXY_CMD;
import org.firstinspires.ftc.teamcode.Auto.autoRobotContainer;
import org.firstinspires.ftc.teamcode.Auto.waitCommand;

@Autonomous
public class driveForward extends autoRobotContainer {

    @Override
    public void path() {

        schedule(new SequentialCommandGroup(

                new MoveRobotEncoderXY_CMD(20,20,3,0.6, driveSub)

        ));
    }
}
