package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class limelightSubsystem extends SubsystemBase {

    Limelight3A limelight;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public limelightSubsystem(HardwareMap hardwareMap){
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }
    public boolean hasTarget(){
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid();
    }
    public double getTx(){ //horizontal offset in degrees
        LLResult r = limelight.getLatestResult();
        return (r != null && r.isValid()) ? r.getTx() : -1.0; //-1 means no target
    }
    public double getTy(){ //vertical offset in degrees
        LLResult r = limelight.getLatestResult();
        return (r != null && r.isValid()) ? r.getTy() : -1.0; //-1 means no target
    }


    public double getDistance(){
        LLResult r = limelight.getLatestResult();

        double targetOffsetAngle_Vertical = r.getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double goalHeightInches = 60.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);



    }


    @Override
    public void periodic() {
        TelemetryPacket llpacket = new TelemetryPacket();
        llpacket.put("Tx: ", limelight.getLatestResult().getTx());
        llpacket.put("Ty: ", limelight.getLatestResult().getTy());
        llpacket.put("Ta: ", limelight.getLatestResult().getTa());
        dashboard.sendTelemetryPacket(llpacket);
    }
}