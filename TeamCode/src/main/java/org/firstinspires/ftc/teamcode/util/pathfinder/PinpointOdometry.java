package org.firstinspires.ftc.teamcode.util.pathfinder;

import android.util.Log;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.robot.AbstractOdometry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

import static org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive.PARAMS;

public class PinpointOdometry extends AbstractOdometry {
    private final GoBildaPinpointDriverRR driver;
    public PinpointOdometry(HardwareMap hardwareMap) {
        driver = hardwareMap.get(GoBildaPinpointDriverRR.class, "odo");
        driver.setOffsets(PARAMS.xOffset, PARAMS.yOffset);
        driver.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);
        driver.setEncoderResolution(PARAMS.resolution);
        Log.d("PinpointDrive", "Initializing Odometry");
        driver.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        driver.setPosition(new Pose2d(0, 0, Math.toRadians(0)));
    }

    @Override
    public void tick() {
        driver.update();
    }

    @Override
    public PointXYZ getRawPosition() {
        Pose2d pose = driver.getPositionRR();
        return new PointXYZ(pose.position.x, pose.position.y, Angle.fromRad(pose.heading.toDouble()));
    }

    public Pose2D getPos() {
        return driver.getPosition();
    }
}
