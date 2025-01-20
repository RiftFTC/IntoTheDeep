package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.ExtendoSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.util.ActionCommand;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.lang.Math;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeV4bSys.*;
import static org.firstinspires.ftc.teamcode.subsystem.OuttakeV4BSys.ARM_HOME;
import static org.firstinspires.ftc.teamcode.subsystem.OuttakeV4BSys.PITCH_HOME;
@Config
@Autonomous(name="Left Auto", group="Autonomous")
public class LeftAuto extends AutoBaseOpMode{

    public static int traj2A = 272;
    TrajectoryActionBuilder preloadDropOffTraj;
    TrajectoryActionBuilder pickUp1Traj;
    TrajectoryActionBuilder dropOff1Traj;
    TrajectoryActionBuilder pickUp2Traj;
    TrajectoryActionBuilder dropOff2Traj;
    TrajectoryActionBuilder pickUp3Traj;
    TrajectoryActionBuilder dropOff3Traj;
    TrajectoryActionBuilder parkTraj;

    Action preloadDropOff;
    Action pickUp1;
    Action dropOff1;
    Action pickUp2;
    Action dropOff2;
    Action pickUp3;
    Action dropOff3;
    Action park;

    @Override
    public void init() {
        super.init();
        drive = new PinpointDrive(hardwareMap, new Pose2d(38.6, 64.5, Math.toRadians(180)));
        telemetry.addData("Initialization", true);
        oClaw.setPosition(0.5);

        preloadDropOffTraj = drive.actionBuilder(
                        new Pose2d(38.6, 64.5, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(54, 53.5), Math.toRadians(225));

        pickUp1Traj = drive.actionBuilder(
                        new Pose2d(54, 53.5, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(51.6, 49.5), Math.toRadians(260));

        dropOff1Traj = drive.actionBuilder(
                        new Pose2d(51.6, 49.5, Math.toRadians(260)))
                .strafeToSplineHeading(new Vector2d(54, 53.5), Math.toRadians(225));

        pickUp2Traj = drive.actionBuilder(
                        new Pose2d(54, 53.5, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(55.5, 50.5), Math.toRadians(traj2A));

        dropOff2Traj = drive.actionBuilder(
                        new Pose2d(57, 50.5, Math.toRadians(traj2A)))
                .strafeToSplineHeading(new Vector2d(54, 53.5), Math.toRadians(225));

        pickUp3Traj = drive.actionBuilder(
                        new Pose2d(54, 53.5, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(60.3, 43.9), Math.toRadians(285));

        dropOff3Traj = drive.actionBuilder(
                        new Pose2d(60.3, 43.9, Math.toRadians(285)))
                .strafeToSplineHeading(new Vector2d(54, 53.5), Math.toRadians(225));

        parkTraj = drive.actionBuilder(
                        new Pose2d(54, 53.5, Math.toRadians(225)))
                .strafeToSplineHeading(new Vector2d(29, 5), Math.toRadians(180));

        preloadDropOff = preloadDropOffTraj.build();
        pickUp1 = pickUp1Traj.build();
        dropOff1 = dropOff1Traj.build();
        pickUp2 = pickUp2Traj.build();
        dropOff2 = dropOff2Traj.build();
        pickUp3 = pickUp3Traj.build();
        dropOff3 = dropOff3Traj.build();
        park = parkTraj.build();

        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ActionCommand(preloadDropOff),
                                liftSys.goTo(LiftSys.HIGH_BUCKET)

                        ),
                        outtakeV4bSys.away(),
                        new WaitCommand(300),
                        outtakeClawSys.release(),
                        new WaitCommand(300),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        outtakeV4bSys.mid(),
                                        liftSys.goTo(LiftSys.NONE),
                                        new ActionCommand(pickUp1),
                                        new SequentialCommandGroup(
                                                intakeClawSys.intake(),
                                                intakeClawSys.release()
                                        )
                                ),
                                new WaitCommand(200),
                                new SequentialCommandGroup(
                                        extendoSys.goTo(0.34),
                                        intakeV4bSys.intake()
                                ),
                                intakeClawSys.release(),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        intakeV4bSys.goToPos(POS_DOWN - 0.03),
                                        new WaitCommand(100),
                                        intakeClawSys.pinch(),
                                        new WaitCommand(200),
                                        intakeClawSys.dropoff(),
                                        intakeV4bSys.dropOff()
                                ),
                                extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
                                outtakeClawSys.release(),
                                new WaitCommand(600),
                                new ParallelCommandGroup(
                                        new ActionCommand(dropOff1),
                                        new SequentialCommandGroup(
                                                outtakeV4bSys.setPitch(PITCH_HOME),
                                                new WaitCommand(100),
                                                outtakeV4bSys.setArm(ARM_HOME),
                                                new WaitCommand(200),
                                                outtakeClawSys.grab(),
                                                new WaitCommand(150),
                                                intakeClawSys.release(),
                                                new WaitCommand(50),
                                                liftSys.goTo(LiftSys.HIGH_BUCKET)
                                        )
                                )
                        ),
                        outtakeV4bSys.away(),
                        new WaitCommand(400),
                        outtakeClawSys.release(),
                        new WaitCommand(200),
                        outtakeV4bSys.mid(),
                        new ParallelCommandGroup(
                                new ActionCommand(pickUp2),
                                liftSys.goTo(LiftSys.NONE)
                        ),
                        new WaitCommand(200),
                        new SequentialCommandGroup(
                                extendoSys.goTo(0.35),
                                intakeV4bSys.intake(),
                                new SequentialCommandGroup(
                                        intakeClawSys.intake(),
                                        intakeClawSys.release(),
                                        intakeClawSys.rotateYaw(0.6)
                                )
                        ),
                        new WaitCommand(200),
                        intakeClawSys.release(),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                intakeV4bSys.goToPos(POS_DOWN - 0.03),
                                new WaitCommand(100),
                                intakeClawSys.pinch(),
                                new WaitCommand(300),
                                intakeClawSys.dropoff(),
                                intakeV4bSys.dropOff()
                        ),
                        extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
                        outtakeClawSys.release(),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new ActionCommand(dropOff2),
                                new SequentialCommandGroup(
                                        outtakeV4bSys.setPitch(PITCH_HOME),
                                        new WaitCommand(100),
                                        outtakeV4bSys.setArm(ARM_HOME),
                                        new WaitCommand(300),
                                        outtakeClawSys.grab(),
                                        new WaitCommand(150),
                                        intakeClawSys.release(),
                                        new WaitCommand(50),
                                        liftSys.goTo(LiftSys.HIGH_BUCKET)
                                )
                        ),
                        new SequentialCommandGroup(
                                outtakeV4bSys.setPitch(1),
                                outtakeV4bSys.setArm(0.3),
                                new WaitCommand(400),
                                outtakeClawSys.release(),
                                new WaitCommand(200),
                                outtakeV4bSys.mid()
                        ),
                        new ParallelCommandGroup(
                                liftSys.goTo(LiftSys.NONE),
                                new ActionCommand(pickUp3),
                                new InstantCommand(pipeline::enableTracking)
                        ),
                        new WaitCommand(300),
                        new SequentialCommandGroup(
                                extendoSys.goTo(0.29),
                                intakeV4bSys.intake(),
                                new SequentialCommandGroup(
                                        intakeClawSys.release()
                                ),
                                new InstantCommand(pipeline::disableTracking),
                                new WaitCommand(400),
                                intakeV4bSys.goToPos(POS_DOWN - 0.03),
                                new WaitCommand(100),
                                intakeClawSys.pinch(),
                                new WaitCommand(300),
                                intakeClawSys.dropoff(),
                                intakeV4bSys.dropOff(),
                                extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
                                new WaitCommand(500)
                        ),
                        new ParallelCommandGroup(
                                new ActionCommand(dropOff3),
                                new SequentialCommandGroup(
                                        outtakeV4bSys.setPitch(PITCH_HOME),
                                        new WaitCommand(100),
                                        outtakeV4bSys.setArm(ARM_HOME),
                                        new WaitCommand(200),
                                        outtakeClawSys.grab(),
                                        new WaitCommand(200),
                                        intakeClawSys.release(),
                                        new WaitCommand(50),
                                        liftSys.goTo(LiftSys.HIGH_BUCKET)
                                )
                        ),
                        new SequentialCommandGroup(
                                outtakeV4bSys.setPitch(1),
                                outtakeV4bSys.setArm(0.3),
                                new WaitCommand(300),
                                outtakeClawSys.release(),
                                new WaitCommand(400)
                        ),
                        new ParallelCommandGroup(
                                outtakeV4bSys.mid(),
                                liftSys.goTo(LiftSys.NONE),
                                new ActionCommand(park),
                                new InstantCommand(pipeline::enableTracking)
                        ),
                        extendoSys.goTo(0.34),
                        intakeV4bSys.goToPos(0.6),
                        intakeV4bSys.goToRoll(0.8),
                        new WaitCommand(400),
                        new InstantCommand(()-> pipeline.getAction(drive,intakeClawSys, intakeV4bSys,extendoSys, outtakeV4bSys, outtakeClawSys, liftSys))
                )
        );
    }

    @Override
    public void loop() {
        liftSys.periodic();
        super.loop();
        Pose2d poseEstimate = drive.odometry.getPositionRR();
        telemetry.addData("x", poseEstimate.position.x);
        telemetry.addData("y", poseEstimate.position.y);
        telemetry.addData("heading", poseEstimate.heading);
    }

}
