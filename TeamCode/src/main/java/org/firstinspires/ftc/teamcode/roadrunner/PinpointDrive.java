package org.firstinspires.ftc.teamcode.roadrunner;

import android.util.Log;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

public class PinpointDrive extends MecanumDrive{
    public static class Params {
        //X OFF = 2.372 in. LEFT
        //Y OFF = 1.8845 in. BACKWARD
        public double xOffset = 60.325;
        public double yOffset = -47.8663;
        public GoBildaPinpointDriverRR.GoBildaOdometryPods resolution = GoBildaPinpointDriverRR.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public GoBildaPinpointDriverRR.EncoderDirection xDirection = GoBildaPinpointDriverRR.EncoderDirection.FORWARD;
        public GoBildaPinpointDriverRR.EncoderDirection yDirection = GoBildaPinpointDriverRR.EncoderDirection.REVERSED;
    }
    public static PinpointDrive.Params PARAMS = new Params();
    public GoBildaPinpointDriverRR odometry;
    private Pose2d lastPinpointPose  = pose;

    public PinpointDrive(HardwareMap hardwareMap, Pose2d initialPose) {
        super(hardwareMap, initialPose);
        odometry = hardwareMap.get(GoBildaPinpointDriverRR.class, "odo");
        odometry.setOffsets(PARAMS.xOffset, PARAMS.yOffset);
        odometry.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
        odometry.setEncoderResolution(PARAMS.resolution);
        Log.d("PinpointDrive", "Initializing Odometry");
        odometry.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        odometry.setPosition(pose);
        Log.d("PinpointDrive", "Odometry Initialized");
    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastPinpointPose != pose) {
            // RR localizer note:
            // Something else is modifying our pose (likely for relocalization),
            // so we override otos pose with the new pose.
            // This could potentially cause up to 1 loop worth of drift.
            // I don't like this solution at all, but it preserves compatibility.
            // The only alternative is to add getter and setters, but that breaks compat.
            // Potential alternate solution: timestamp the pose set and backtrack it based on speed?
            odometry.setPosition(pose);
        }
        odometry.update();
        pose = odometry.getPositionRR();
        lastPinpointPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));

        return odometry.getVelocityRR();
    }
}