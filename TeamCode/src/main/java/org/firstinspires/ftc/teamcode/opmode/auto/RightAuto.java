package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.ExtendoSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.util.ActionCommand;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeV4bSys.POS_DOWN;

@Autonomous(name="RightAuto", group="Autonomous")
public class RightAuto extends AutoBaseOpMode{

    TrajectoryActionBuilder preloadDropOffTraj;

    Action preloadDropOff;
    Action pickupOffField1;
    Action dropOffField1;
    Action pickupOffField2;
    Action dropOffField2;
    Action dropOffField3;
    Action pickUp1;
    Action pickUp1F;
    Action dropOff1;
    Action pickUp2;
    Action pickUp2F;
    Action dropOff2;
    Action pickUp3;
    Action dropOff3;
    Action park;

    @Override
    public void init() {
        super.init();
        drive = new PinpointDrive(hardwareMap, new Pose2d(-16.6, 63, Math.toRadians(90)));
        telemetry.addData("Initialization", true);
        oPos.setPosition(0.55);
        oPitch.setPosition(0.7);
        iClaw.setPosition(0.63);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        oClaw.setPosition(0.7);
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        oClaw.setPosition(0.0);

        preloadDropOffTraj = drive.actionBuilder(new Pose2d(-16.6, 63, Math.toRadians(90)))
                .strafeTo(new Vector2d(-5.7, 36.1));

        preloadDropOff = preloadDropOffTraj.build();

        pickupOffField1 = drive.actionBuilder(new Pose2d(-5.7, 36.1, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-27, 36.8 ), Math.toRadians(205)).build();

        dropOffField1 = drive.actionBuilder(new Pose2d(-27, 36.8, Math.toRadians(205)))
                .turnTo(Math.toRadians(120)).build();

        pickupOffField2 = drive.actionBuilder(new Pose2d(-27, 36.8, Math.toRadians(120)))
                .strafeToSplineHeading(new Vector2d(-35.1, 33.0), Math.toRadians(200)).build();

        dropOffField2 = drive.actionBuilder(new Pose2d(-35.1, 33.0, Math.toRadians(200)))
                .turnTo(Math.toRadians(120)).build();

        dropOffField3 = drive.actionBuilder(new Pose2d(-35.1, 33.0, Math.toRadians(120)))
                .strafeToSplineHeading(new Vector2d(-62.0, 11.8), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-62, 44), Math.toRadians(270))
                .strafeToSplineHeading(new Vector2d(-38.6, 59.7), Math.toRadians(270))
                .build();

        dropOff1 = drive.actionBuilder(new Pose2d(-38.6, 61, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-5.7, 36.1), Math.toRadians(90)).build();

        pickUp2 = drive.actionBuilder(new Pose2d(-5.7, 36.1, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(-38.6, 59.7), Math.toRadians(270)).build();

        pickUp2F = drive.actionBuilder(new Pose2d(-38.6, 59.7, Math.toRadians(270)))
                .strafeToSplineHeading(new Vector2d(-38.6, 61), Math.toRadians(270)).build();

        park = drive.actionBuilder(new Pose2d(-5.7, 36.1, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-27.1, 32.09), Math.toRadians(270)).build();

        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ActionCommand(preloadDropOff),
                                liftSys.goTo(LiftSys.LOW_BUCKET)
                        ),
                        outtakeV4bSys.setArm(0.23),
                        outtakeV4bSys.setPitch(0.8),
                        new WaitCommand(400),
                        liftSys.goTo(LiftSys.LOW_BUCKET - 320),
                        outtakeClawSys.superRelease(),
                        new WaitCommand(200),
                        outtakeV4bSys.away(),
                        new ParallelCommandGroup(
                                new ActionCommand(pickupOffField1),
                                liftSys.goTo(LiftSys.NONE)
                        ),
                        new WaitCommand(300),
                        new SequentialCommandGroup(
                                extendoSys.goTo(0.43),
                                intakeV4bSys.intake(),
                                intakeClawSys.rotateYaw(0),
                                intakeClawSys.release(),
                                new WaitCommand(400),
                                intakeV4bSys.intake(),
                                new WaitCommand(400),
                                intakeV4bSys.goToPos(POS_DOWN),
                                new WaitCommand(100),
                                intakeClawSys.pinch(),
                                new WaitCommand(100),
                                intakeClawSys.dropoff(),
                                intakeV4bSys.dropOff(),
                                extendoSys.goTo(ExtendoSys.EXTENDO_HOME)
                        ),
                        new WaitCommand(300),
                        new ActionCommand(dropOffField1),
                        extendoSys.goTo(0.43),
                        intakeV4bSys.intake(),
                        new WaitCommand(300),
                        intakeClawSys.release(),
                        new WaitCommand(300),
                        intakeClawSys.dropoff(),
                        intakeV4bSys.dropOff(),
                        extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
//                        new ActionCommand(pickupOffField2),
//                        extendoSys.goTo(0.43),
//                        intakeV4bSys.intake(),
//                        intakeClawSys.rotateYaw(0),
//                        intakeClawSys.release(),
//                        new WaitCommand(300),
//                        intakeV4bSys.goToPos(POS_DOWN),
//                        new WaitCommand(100),
//                        intakeClawSys.pinch(),
//                        new WaitCommand(200),
//                        intakeClawSys.dropoff(),
//                        intakeV4bSys.dropOff(),
//                        extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
//                        new WaitCommand(400),
//                        new ActionCommand(dropOffField2),
//                        extendoSys.goTo(0.46),
//                        intakeV4bSys.intake(),
//                        new WaitCommand(400),
//                        intakeClawSys.release(),
//                        new WaitCommand(300),
//                        intakeClawSys.dropoff(),
//                        intakeV4bSys.dropOff(),
//                        extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
//                        new WaitCommand(300),
                        new ActionCommand(dropOffField3),
                        new WaitCommand(200),
                        outtakeClawSys.grab(),
                        new WaitCommand(300),
                        new ParallelCommandGroup(new ActionCommand(pickUp2F), liftSys.goTo(LiftSys.NONE + 50)),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new ActionCommand(dropOff1),
                                liftSys.goTo(LiftSys.LOW_BUCKET),
                                outtakeV4bSys.home()
                        ),
                        outtakeV4bSys.setPitch(0.8),
                        outtakeV4bSys.setArm(0.23),
                        new WaitCommand(500),
                        liftSys.goTo(LiftSys.LOW_BUCKET - 320),
                        outtakeClawSys.superRelease(),
                        new WaitCommand(200),
                        outtakeV4bSys.away(),
                        new ParallelCommandGroup(
                                new ActionCommand(pickUp2),
                                liftSys.goTo(LiftSys.NONE)
                        ),
                        new WaitCommand(200),
                        outtakeClawSys.grab(),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new ActionCommand(pickUp2F),
                                liftSys.goTo(LiftSys.NONE + 50)
                        ),
                        new ParallelCommandGroup(
                                new ActionCommand(dropOff1),
                                liftSys.goTo(LiftSys.LOW_BUCKET),
                                outtakeV4bSys.home()
                        ),
                        outtakeV4bSys.setPitch(0.8),
                        outtakeV4bSys.setArm(0.23),
                        new WaitCommand(400),
                        liftSys.goTo(LiftSys.LOW_BUCKET - 320),
                        outtakeClawSys.superRelease(),
                        new WaitCommand(200),
                        outtakeV4bSys.away(),
                        new ParallelCommandGroup(
                                new ActionCommand(pickUp2),
                                liftSys.goTo(LiftSys.NONE)
                        ),
                        new WaitCommand(200),
                        outtakeClawSys.grab(),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new ActionCommand(dropOff1),
                                liftSys.goTo(LiftSys.LOW_BUCKET),
                                outtakeV4bSys.home()
                        ),
                        outtakeV4bSys.setPitch(0.8),
                        outtakeV4bSys.setArm(0.23),
                        new WaitCommand(400),
                        liftSys.goTo(LiftSys.LOW_BUCKET - 320),
                        outtakeClawSys.superRelease(),
                        new WaitCommand(200),
                        outtakeV4bSys.away(),
                        new ActionCommand(park)
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
