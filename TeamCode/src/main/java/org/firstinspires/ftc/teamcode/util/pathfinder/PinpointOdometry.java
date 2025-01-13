package org.firstinspires.ftc.teamcode.util.pathfinder;

import com.qualcomm.robotcore.hardware.HardwareMap;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.robot.AbstractOdometry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.math.Pose2D;

public class PinpointOdometry extends AbstractOdometry {
    private GoBildaPinpointDriver driver;
    public PinpointOdometry(HardwareMap hardwareMap) {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }

    @Override
    public PointXYZ getRawPosition() {
        Pose2D pose = driver.getPosition();
        return new PointXYZ(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
    }
}
