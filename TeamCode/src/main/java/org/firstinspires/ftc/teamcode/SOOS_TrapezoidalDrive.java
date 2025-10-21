package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.drivetrain_ka;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_ks;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_kv;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_maxAccel;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_maxVelocity;
import static org.firstinspires.ftc.teamcode.Constants.drivetrain_targetDistance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.mecanumDriveSubsystem;

@TeleOp (name = "SOOS_TrapezoidalDrive")
public class SOOS_TrapezoidalDrive extends OpMode {
    private mecanumDriveSubsystem drive;
    SparkFunOTOS myOtos;
    GamepadEx gamepadEx;
    TelemetryPacket packet = new TelemetryPacket();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Motion profile parameters
    private final double maxAccel = drivetrain_maxAccel; // inches/sec^2
    private final double maxVel = drivetrain_maxVelocity;   // inches/sec
    private final double targetDistance = drivetrain_targetDistance; // inches

    // Feedforward Constants
    private final double Kv = drivetrain_kv;
    private final double Ka = drivetrain_ka;
    private final double Ks = drivetrain_ks;

    //times
    private ElapsedTime timer = new ElapsedTime();
    private double totalTime;
    private double accelTime;
    private double cruiseTime;

    //Current phase
    private enum Phase { ACCEL, CRUISE, DECEL, DONE }
    private Phase currentPhase = Phase.ACCEL;


    @Override
    public void init(){
        //assigning names
        gamepadEx = new GamepadEx(gamepad1);
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // Calculate profile timing once
        accelTime = maxVel / maxAccel;
        double accelDist = 0.5 * maxAccel * accelTime * accelTime;
        if (2 * accelDist > targetDistance) {
            accelTime = Math.sqrt(targetDistance / maxAccel);
        }
        double cruiseDist = targetDistance - 2 * (0.5 * maxAccel * accelTime * accelTime);
        cruiseTime = cruiseDist / maxVel;
        totalTime = 2 * accelTime + cruiseTime;
        //reset timer
        timer.reset();
    }


    @Override
    public void loop() {
        double t = timer.seconds();
        double desiredVel = 0;
        double desiredAccel = 0;

        // --- Trapezoid calculation ---
        if (t < accelTime) { // accelerating
            desiredAccel = maxAccel;
            desiredVel = maxAccel * t;
            currentPhase = Phase.ACCEL;
        } else if (t < accelTime + cruiseTime) { // cruising
            desiredAccel = 0;
            desiredVel = maxVel;
            currentPhase = Phase.CRUISE;
        } else if (t < totalTime) { // decelerating
            desiredAccel = -maxAccel;
            double timeSinceDecel = t - (accelTime + cruiseTime);
            desiredVel = maxVel - maxAccel * timeSinceDecel;
            currentPhase = Phase.DECEL;
        } else { // done
            desiredAccel = 0;
            desiredVel = 0;
            currentPhase = Phase.DONE;
        }

        // --- Feedforward motor power ---
        double feedforward = Ks + Kv * desiredVel + Ka * desiredAccel;

        // Stop once the sensor reads we've moved ~targetDistance
        double distanceTraveled = Math.hypot(myOtos.getPosition().x, myOtos.getPosition().y);
        if (distanceTraveled >= targetDistance) {
            feedforward = 0;
            currentPhase = Phase.DONE;
        }

        // When button A is down feedforward is sent to the motors to go forward
        if(gamepadEx.isDown(GamepadKeys.Button.A)) {
            drive.drive(feedforward, 0, 0, true);
        }
        // --- Telemetry ---
        packet.put("Phase", currentPhase);
        packet.put("Time", t);
        packet.put("Desired Velocity", desiredVel);
        packet.put("Desired Accel", desiredAccel);
        packet.put("Feedforward", feedforward);
        packet.put("Distance", distanceTraveled);
        dashboard.sendTelemetryPacket(packet);
    }




}
