package org.firstinspires.ftc.teamcode.Auto.Paths;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.MoveRobotEncoderXY_CMD;
import org.firstinspires.ftc.teamcode.Auto.autoRobotContainer;

@Autonomous
public class driveForward extends autoRobotContainer {

    @Override
    public void path() {

        schedule(new SequentialCommandGroup(

                new MoveRobotEncoderXY_CMD(15,15,3,0.4, driveSub)
        ));
    }
}
