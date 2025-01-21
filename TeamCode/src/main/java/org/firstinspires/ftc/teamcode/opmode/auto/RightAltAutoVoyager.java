package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.ExtendoSys;
import org.firstinspires.ftc.teamcode.subsystem.IntakeV4bSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.util.ActionCommand;
import org.firstinspires.ftc.teamcode.util.VoyagerCommand;
import org.firstinspires.ftc.teamcode.util.pathfinder.PathfinderRobot;
import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name="Right ALT Auto w/ Voyager")
public class RightAltAutoVoyager extends AutoBaseOpMode{
    PathfinderRobot pathfinderRobot = new PathfinderRobot();
    Action preLoad;
    Action dropOff;
    Action score1;
    Action pickUp2;
    Action score2;
    Action pickUp3;
    Action score3;
    Action park;

    private final List<PointXYZ> dropOffVoyager = new ArrayList<PointXYZ>() {{
        add(new PointXYZ(34, -33, Angle.fromDeg(90)));
        add(new PointXYZ(34, -10, Angle.fromDeg(90)));
        add(new PointXYZ(50, -10, Angle.fromDeg(90))); //-47
        add(new PointXYZ(50, -47, Angle.fromDeg(90)));
        add(new PointXYZ(50, -10, Angle.fromDeg(90)));
        add(new PointXYZ(57,-10, Angle.fromDeg(90)));
        add(new PointXYZ(57,-47, Angle.fromDeg(90)));
        add(new PointXYZ(34.7, -57.3, Angle.fromDeg(90)));
    }};

    public static double pickupN = -57.3;
    public static double dropOffN = -36.8;

    @Override
    public void init() {
        super.init();
        pathfinderRobot.init(hardwareMap);
        Voyager voyager = pathfinderRobot.voyager();
        voyager.setSpeed(1);
        voyager.setAngleTolerance(Angle.fromDeg(3));
        voyager.setTolerance(1.5);
        drive = new PinpointDrive(hardwareMap, new Pose2d(16.7, -62.2, Math.toRadians(270)));
        telemetry.addData("Initialization", true);
        telemetry.update();
        oClaw.setPosition(0.8);

        preLoad = drive.actionBuilder(new Pose2d(16.7, -62.2, Math.toRadians(270)))
                .strafeTo(new Vector2d(4, dropOffN), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        dropOff = drive.actionBuilder(new Pose2d(4, dropOffN, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(20, -36), Math.toRadians(90))
                .build();

        score1 = drive.actionBuilder(new Pose2d(34.7, pickupN, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(4, dropOffN), Math.toRadians(270),drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        pickUp2 = drive.actionBuilder(new Pose2d(4, dropOffN, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(34.7, pickupN), Math.toRadians(90), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        score2 = drive.actionBuilder(new Pose2d(34.7, pickupN, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(6, dropOffN), Math.toRadians(270), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        pickUp3 = drive.actionBuilder(new Pose2d(6, dropOffN, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(34.7, pickupN), Math.toRadians(90), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        score3 = drive.actionBuilder(new Pose2d(34.7, pickupN, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(8, dropOffN), Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();

        park = drive.actionBuilder(new Pose2d(8, dropOffN, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(24, -45), Math.toRadians(315), drive.defaultVelConstraint, drive.maxAccelConstraint)
                .build();

        schedule(
                new SequentialCommandGroup(
                        // SCORE PRELOAD
                        new ParallelCommandGroup(
                                new ActionCommand(preLoad),
                                liftSys.goTo(LiftSys.HIGH_RUNG)
                        ),
                        outtakeV4bSys.away(),
                        new WaitCommand(250),
                        //outtake v4b sm shit
                        liftSys.goTo(LiftSys.HIGH_RUNG - 400),
                        new WaitCommand(200),
                        outtakeClawSys.release(),
                        new WaitCommand(150),
                        outtakeV4bSys.mid(),
                        //DROP OFF 1
                        new InstantCommand(()->drive.setTolerance(1, 3)),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new ActionCommand(dropOff),
                                        new VoyagerCommand(voyager, dropOffVoyager)
                                ),
                                intakeClawSys.release(),
                                outtakeClawSys.release(),
                                new SequentialCommandGroup(
                                        liftSys.goTo(LiftSys.NONE),
                                        outtakeV4bSys.mid()
                                )
                        ),
                        //SCORE 1
                        outtakeV4bSys.specimen(),
                        new WaitCommand(500),
                        outtakeClawSys.grab(),
                        new WaitCommand(150),
                        outtakeV4bSys.mid(),
                        new ParallelCommandGroup(
                                new ActionCommand(score1),
                                new SequentialCommandGroup(
                                        liftSys.goTo(LiftSys.HIGH_RUNG)
                                )
                        ),
                        outtakeV4bSys.away(),
                        //outtake v4b some shit
                        liftSys.goTo(LiftSys.HIGH_RUNG-400),
                        new WaitCommand(200),
                        outtakeClawSys.release(),
                        new WaitCommand(100),
                        //PICKUP 2
                        new ParallelCommandGroup(
                                new ActionCommand(pickUp2),
                                new SequentialCommandGroup(
                                        liftSys.goTo(LiftSys.NONE),
                                        outtakeV4bSys.specimen()
                                )
                        ),
                        new WaitCommand(200),
                        outtakeClawSys.grab(),
                        new WaitCommand(200),
                        outtakeV4bSys.mid(),
                        new ParallelCommandGroup(
                                new ActionCommand(score2),
                                liftSys.goTo(LiftSys.HIGH_RUNG)
                        ),
                        outtakeV4bSys.away(),
                        //score sm shit
                        liftSys.goTo(LiftSys.HIGH_RUNG - 400),
                        outtakeClawSys.release(),
                        new WaitCommand(150),
                        //PICKUP 3
                        new ParallelCommandGroup(
                                new ActionCommand(pickUp3),
                                new SequentialCommandGroup(
                                        liftSys.goTo(LiftSys.NONE),
                                        outtakeV4bSys.specimen()
                                )

                        ),
                        new WaitCommand(200),
                        outtakeClawSys.grab(),
                        new WaitCommand(200),
                        outtakeV4bSys.mid(),
                        new ParallelCommandGroup(
                                new ActionCommand(score3),
                                liftSys.goTo(LiftSys.HIGH_RUNG)
                        ),
                        outtakeV4bSys.away(),
                        liftSys.goTo(LiftSys.HIGH_RUNG - 400),
                        outtakeClawSys.release(),
                        new WaitCommand(150),
                        //PARK
                        new ParallelCommandGroup(
                                new InstantCommand(()->drive.setTolerance(4, 0)),
                                new ActionCommand(park),
                                liftSys.goTo(LiftSys.NONE),
                                outtakeV4bSys.mid()
                        ),
                        extendoSys.goTo(ExtendoSys.EXTENDO_MAX),
                        intakeV4bSys.goToPos(IntakeV4bSys.POS_DOWN),
                        intakeV4bSys.goToRoll(IntakeV4bSys.ROLL_OUT),
                        new InstantCommand(()->drive.setTolerance(1, 4))
                )
        );
    }

    @Override
    public void loop() {
        pathfinderRobot.voyager().getOdometry().tick();
        drive.updatePoseEstimate();
        liftSys.periodic();
        telemetry.addData("Voyager", pathfinderRobot.voyager().getOdometry().getPosition().toString());
        super.loop();
        Pose2d poseEstimate = drive.odometry.getPositionRR();
        Robot.startPose = poseEstimate;
        telemetry.addData("x", poseEstimate.position.x);
        telemetry.addData("y", poseEstimate.position.y);
        telemetry.addData("heading", poseEstimate.heading);
    }
}
