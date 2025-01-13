package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.ExtendoSys;
import org.firstinspires.ftc.teamcode.subsystem.IntakeV4bSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.util.ActionCommand;

@TeleOp(name="Right SPEED Auto")
public class RightSpeedAuto extends AutoBaseOpMode{
    Action preLoad;
    Action dropOffLocation;
    Action dropOff2;
    Action dropOff3;
    Action pickup1;
    Action score1;
    Action pickUp2;
    Action score2;
    Action pickUp3;
    Action score3;
    Action park;

    @Override
    public void init() {
        super.init();
        drive = new PinpointDrive(hardwareMap, new Pose2d(16.7, -62.2, Math.toRadians(270)));
        telemetry.addData("Initialization", true);
        telemetry.update();
        oClaw.setPosition(0.3);

        preLoad = drive.actionBuilder(new Pose2d(16.7, -62.2, Math.toRadians(270)))
                .strafeTo(new Vector2d(4, -34.9), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        dropOffLocation = drive.actionBuilder(new Pose2d(4, -34.9, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(58.7, -50), Math.toRadians(115))
                .build();

        dropOff2 = drive.actionBuilder(new Pose2d(58.7,-50, Math.toRadians(115)))
                .strafeToLinearHeading(new Vector2d(58.7, -50), Math.toRadians(90))
                .build();

        dropOff3 = drive.actionBuilder(new Pose2d(58.7,-50,Math.toRadians(70)))
                .strafeToLinearHeading(new Vector2d(58.7, -50),Math.toRadians(70))
                .build();

        pickup1 = drive.actionBuilder(new Pose2d(58.7,-50,Math.toRadians(70)))
                .strafeToLinearHeading(new Vector2d(34.7, -60), Math.toRadians(90))
                .build();

        score1 = drive.actionBuilder(new Pose2d(34.7, -60, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(6, -34.9), Math.toRadians(270),drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        pickUp2 = drive.actionBuilder(new Pose2d(6, -34.9, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(34.7, -60), Math.toRadians(90), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        score2 = drive.actionBuilder(new Pose2d(34.7, -60, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(8, -34.9), Math.toRadians(270), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        pickUp3 = drive.actionBuilder(new Pose2d(8, -34.9, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(34.7, -60), Math.toRadians(90), drive.defaultVelConstraint, drive.defaultAccelConstraint)
                .build();

        score3 = drive.actionBuilder(new Pose2d(34.7, -60, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(10, -34.9), Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();

        park = drive.actionBuilder(new Pose2d(10, -34.9, Math.toRadians(270)))
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
                        //outtake v4b sm shit
                        liftSys.goTo(LiftSys.HIGH_RUNG - 200),
                        outtakeClawSys.release(),
                        new WaitCommand(200),
                        //DROP OFF 1
                        new ParallelCommandGroup(
                                intakeClawSys.release(),
                                outtakeClawSys.release(),
                                new ActionCommand(dropOffLocation)
                        ),
                        new WaitCommand(150),
                        new ActionCommand(dropOff2),
                        new ActionCommand(dropOff3),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        extendoSys.goTo(ExtendoSys.EXTENDO_MAX),
                                        new SequentialCommandGroup(
                                                intakeV4bSys.goToPos(IntakeV4bSys.POS_DOWN),
                                                intakeV4bSys.goToRoll(IntakeV4bSys.ROLL_OUT)
                                        )
                                ),
                                new WaitCommand(250),
                                intakeClawSys.pinch(),
                                new WaitCommand(100),
                                new ParallelCommandGroup(
                                        extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
                                        intakeV4bSys.dropOff()
                                ),
                                new WaitCommand(300),
                                outtakeV4bSys.home(),
                                new WaitCommand(100),
                                outtakeClawSys.grab(),
                                new WaitCommand(150),
                                outtakeV4bSys.away(),
                                new WaitCommand(50),
                                outtakeClawSys.release()
                        ),
                        new ActionCommand(pickup1),
                        //SCORE 1
                        outtakeV4bSys.away(),
                        new WaitCommand(150),
                        outtakeClawSys.grab(),
                        new WaitCommand(150),
                        outtakeV4bSys.mid(),
                        new ParallelCommandGroup(
                                new ActionCommand(score1),
                                liftSys.goTo(LiftSys.HIGH_RUNG)
                        ),
                        outtakeV4bSys.away(),
                        //outtake v4b some shit
                        liftSys.goTo(LiftSys.HIGH_RUNG-200),
                        new WaitCommand(200),
                        outtakeClawSys.release(),
                        new WaitCommand(100),
                        //PICKUP 2
                        new ParallelCommandGroup(
                                new ActionCommand(pickUp2),
                                liftSys.goTo(LiftSys.NONE),
                                outtakeV4bSys.mid()

                        ),
                        outtakeV4bSys.away(),
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
                        liftSys.goTo(LiftSys.HIGH_RUNG - 200),
                        outtakeClawSys.release(),
                        new WaitCommand(150),
                        //PICKUP 3
                        new ParallelCommandGroup(
                                new ActionCommand(pickUp3),
                                liftSys.goTo(LiftSys.NONE),
                                outtakeV4bSys.mid()
                        ),
                        outtakeV4bSys.away(),
                        new WaitCommand(200),
                        outtakeClawSys.grab(),
                        new WaitCommand(200),
                        outtakeV4bSys.mid(),
                        new ParallelCommandGroup(
                                new ActionCommand(score3),
                                liftSys.goTo(LiftSys.HIGH_RUNG)

                        ),
                        outtakeV4bSys.away(),
                        liftSys.goTo(LiftSys.HIGH_RUNG - 200),
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
        liftSys.periodic();
        super.loop();
        Pose2d poseEstimate = drive.odometry.getPositionRR();
        telemetry.addData("x", poseEstimate.position.x);
        telemetry.addData("y", poseEstimate.position.y);
        telemetry.addData("heading", poseEstimate.heading);
    }
}
