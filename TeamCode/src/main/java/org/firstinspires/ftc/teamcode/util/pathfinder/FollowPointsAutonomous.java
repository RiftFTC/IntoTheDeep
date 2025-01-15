package org.firstinspires.ftc.teamcode.util.pathfinder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.geometry.Translation;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class FollowPointsAutonomous extends LinearOpMode {
    private final List<PointXYZ> points = new ArrayList<PointXYZ>() {{
        add(new PointXYZ(20, 20, Angle.fromDeg(0)));
//        add(new PointXYZ(15, 20, Angle.fromDeg(90)));
//        add(new PointXYZ(30, 10, Angle.fromDeg(90)));
//        add(new PointXYZ(30, 10, Angle.fromDeg(180)));
    }};

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize pathfinder and the pathfinder robot
        PathfinderRobot pathfinderRobot = new PathfinderRobot();
        pathfinderRobot.init(hardwareMap);
        Voyager voyager = pathfinderRobot.pathfinder();

        voyager.setSpeed(1);
        voyager.setAngleTolerance(Angle.fromDeg(20));
        voyager.setTolerance(10);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // wait for the game to start
        waitForStart();

        // while the op mode is going on
        while (opModeIsActive()) {
            pathfinderRobot.update();

            telemetry.addData("Position", voyager.getOdometry().getPosition().toString());
//            Pose2D poseEstimate = pathfinderRobot.pos();
//            telemetry.addData("x", poseEstimate.getX(DistanceUnit.INCH));
//            telemetry.addData("y", poseEstimate.getY(DistanceUnit.INCH));
//            telemetry.addData("heading", poseEstimate.getHeading(AngleUnit.DEGREES));
//            poseEstimate = pinpoint.getPos();
//            telemetry.addData("pinpoint x",poseEstimate.getX(DistanceUnit.INCH));
//            telemetry.addData("pinpoint y", poseEstimate.getY(DistanceUnit.INCH));
//            telemetry.addData("pinpoint heading ", poseEstimate.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("pinpoint", pinpoint.getPosition());
            telemetry.addData("tolerance", voyager.getPosition().isNear(new PointXYZ(20,-20,Angle.fromDeg(0)), 5, Angle.fromDeg(15)));

            if (!voyager.getPosition().isNear(new PointXYZ(20,-20,Angle.fromDeg(0)), 5, Angle.fromDeg(15))) {
                if (voyager.isActive()) {
                    // if pathfinder is active (meaning it's going somewhere)
                    // we want to tick/update it
                    voyager.tick();
                } else {
                    // pathfinder's not active - either...
                    // 1. there are more points to visit
                    // 2. we've finished
                    // if there's more points, we should go to the next point. if
                    // there are no more points, we should stop the robot.
                    if (!points.isEmpty()) {

                        PointXYZ nextPoint = points.get(0);
                        voyager.goTo(nextPoint);
                        points.remove(nextPoint);
                    } else {
                         // stop the robot
                        voyager.getDrive().setTranslation(Translation.zero());
                    }
                }
            } else {
                voyager.getDrive().setTranslation(Translation.zero());
            }

            telemetry.update();
        }
    }

}