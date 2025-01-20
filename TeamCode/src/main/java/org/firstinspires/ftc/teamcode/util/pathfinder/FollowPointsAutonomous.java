package org.firstinspires.ftc.teamcode.util.pathfinder;

import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.*;
import xyz.devmello.voyager.pathgen.LocalizedPathGen;
import xyz.devmello.voyager.zones.Zone;
import java.util.ArrayList;
import java.util.List;

@Autonomous
public class FollowPointsAutonomous extends LinearOpMode {
    private final List<PointXYZ> points = new ArrayList<PointXYZ>() {{
        //add(new PointXYZ(0, 20, Angle.fromDeg(0)));
        add(new PointXYZ(20, 0, Angle.fromDeg(0)));
        add(new PointXYZ(20, -20, Angle.fromDeg(180)));
        add(new PointXYZ(0, -20, Angle.fromDeg(270)));
        add(new PointXYZ(0,0,Angle.fromDeg(0)));
    }};

    @Override
    public void runOpMode() throws InterruptedException {
        List<Zone> zones = new ArrayList<>();
        zones.add(new Zone(new Rectangle(new PointXY(15,10), new PointXY(20,10), new PointXY(20,20), new PointXY(15,20)), () -> {
            Log.i("Zone", "Entered");}, () -> {}, () -> {}));
        LocalizedPathGen pathGen = new LocalizedPathGen(zones, 1,1);
        List<PointXY> path = pathGen.getPath(new PointXY(0,0), new PointXY(40,30));
        for(PointXY point : path) {
            Log.i("Point", point.toString());
            points.add(new PointXYZ(point, Angle.fromDeg(0)));
        }
        // initialize pathfinder and the pathfinder robot
        PathfinderRobot pathfinderRobot = new PathfinderRobot();
        pathfinderRobot.init(hardwareMap);
        Voyager voyager = pathfinderRobot.voyager();

        voyager.setSpeed(0.5);
        voyager.setAngleTolerance(Angle.fromDeg(4));
        voyager.setTolerance(1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // wait for the game to start
        waitForStart();

        // while the op mode is going on
        while (opModeIsActive()) {
            telemetry.addData("Position", voyager.getOdometry().getPosition().toString());

            telemetry.addData("tolerance", voyager.getPosition().isNear(new PointXYZ(20,-20,Angle.fromDeg(0)), 5, Angle.fromDeg(15)));
            //!voyager.getPosition().isNear(new PointXYZ(20,-20,Angle.fromDeg(0)), 5, Angle.fromDeg(15))
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
                    //if (points.size() == 1) voyager.lockHeading(Angle.fromDeg(90));
                } else {
                    // stop the robot
                    voyager.getDrive().setTranslation(Translation.zero());
                }
            }
            telemetry.update();
        }
    }

}