package org.firstinspires.ftc.teamcode.util.pathfinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.geometry.Translation;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class FollowPointsAutonomous extends LinearOpMode {
    private final List<PointXYZ> points = new ArrayList<PointXYZ>() {{
        add(new PointXYZ(10, 10, Angle.fromDeg(45)));
        add(new PointXYZ(15, 20, Angle.fromDeg(90)));
        add(new PointXYZ(30, 10, Angle.fromDeg(90)));
        add(new PointXYZ(30, 10, Angle.fromDeg(180)));
    }};

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize pathfinder and the pathfinder robot
        PathfinderRobot pathfinderRobot = new PathfinderRobot();
        pathfinderRobot.init(hardwareMap);
        Voyager voyager = pathfinderRobot.pathfinder();

        // wait for the game to start
        waitForStart();

        // while the op mode is going on
        while (opModeIsActive()) {
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
                    voyager.getDrive().setTranslation(Translation.zero()); // stop the robot
                }
            }

        }
    }
}