package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Command.findTPSCmd;
import org.firstinspires.ftc.teamcode.Command.shooterAdaptCmd;
import org.firstinspires.ftc.teamcode.Command.teleOpMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.Subsystem.cShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.limelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

@TeleOp(name = "TeleOpMode")
public class RobotContainer extends CommandOpMode {
//    private mecanumDriveSubsystem driveSub;

    private cShooterSubsystem shooterSub;

    private limelightSubsystem llsub;

    private GamepadEx driverJoystick;

    @Override
    public void initialize() {

//        // Mecanum Motor binding
//        driveSub = new mecanumDriveSubsystem(
//                hardwareMap.get(DcMotor.class,"front_left"),
//                hardwareMap.get(DcMotor.class, "front_right"),
//                hardwareMap.get(DcMotor.class, "back_left"),
//                hardwareMap.get(DcMotor.class, "back_right"),
//                hardwareMap
//        );


        shooterSub = new cShooterSubsystem(
                hardwareMap.get(DcMotorEx.class, "shootermotor1")
        );

        llsub = new limelightSubsystem(
                hardwareMap
        );





        driverJoystick = new GamepadEx(gamepad1);

        runCommands();
        setDefaultCommands();





    }

    /**
     * Apply a joystick deadband so tiny inputs don’t move the motors.
     *
     * @param value     joystick value
     * @param threshold minimum absolute value to count as input
     * @return filtered value
     */
    private double applyDeadband(double value, double threshold) {
        return (Math.abs(value) > threshold) ? value : 0.0;
    }

    public void setDefaultCommands() {
        /*
         * Sets the joysticks to always work to drive the robot
//         * unless a different Op mode is selected
//         */
//        driveSub.setDefaultCommand(
//               new teleOpMecanumDriveCommand(
//                        driveSub,
//                        () -> applyDeadband(driverJoystick.getLeftY(), 0.05),  // Forward/back
//                        () -> applyDeadband(driverJoystick.getLeftX(), 0.05),  // Strafe
//                        () -> applyDeadband(driverJoystick.getRightX(), 0.05)  // Rotate
//                )
//        );
//




        Trigger shooter_Trigger = new Trigger(() -> {
            return driverJoystick.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
        });

        shooter_Trigger.whileActiveContinuous(new findTPSCmd(shooterSub));










    }

    private void runCommands() {
        // Add other commands here if needed


    }
}