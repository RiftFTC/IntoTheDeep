package org.firstinspires.ftc.teamcode.roadrunner;

import android.util.Log;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.math.Precision;

public class PinpointDriveDev extends MecanumDrive{
    public static class Params {
        //X OFF = 2.372 in. LEFT
        //Y OFF = 1.8845 in. BACKWARD
        public double xOffset = 60.325;
        public double yOffset = -47.8663;
        public GoBildaPinpointDriver.GoBildaOdometryPods resolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
    public static PinpointDriveDev.Params PARAMS = new Params();
    public GoBildaPinpointDriver odometry;  //Odometry
    private Pose2d lastPose = pose;

    public PinpointDriveDev(HardwareMap hardwareMap, Pose2d initialPose) {
        super(hardwareMap, initialPose);
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odometry.setOffsets(PARAMS.xOffset, PARAMS.yOffset);
        odometry.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
        odometry.setEncoderResolution(PARAMS.resolution);
        Log.d("PinpointDrive", "Initializing Odometry");
        odometry.resetPosAndIMU();
        Log.d("PinpointDrive", "Odometry Initialized");
    }

    @Override
    public PoseVelocity2d updatePoseEstimate() {
        odometry.update();
        if (lastPose != pose) odometry.setPosition(Precision.fromPose2d(pose));
        pose = Precision.toPose2d(odometry.getPosition());
        lastPose = pose;
        poseHistory.add(pose);
        while (poseHistory.size() > 100) poseHistory.pop();
        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        Log.d("PinpointDrive", "Updating Pose Estimate");
        Log.d("PinpointDrive", "Pose: " + odometry.getPosition().getX(DistanceUnit.INCH) + ", " + odometry.getPosition().getY(DistanceUnit.INCH) + ", " + odometry.getHeading());
        return new PoseVelocity2d(new Vector2d(odometry.getVelocity().getX(DistanceUnit.INCH), odometry.getVelocity().getY(DistanceUnit.INCH)), odometry.getHeadingVelocity());
    }
}
