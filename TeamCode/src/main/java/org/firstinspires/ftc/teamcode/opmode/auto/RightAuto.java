package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.ExtendoSys;
import org.firstinspires.ftc.teamcode.subsystem.LiftSys;
import org.firstinspires.ftc.teamcode.util.ActionCommand;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeV4bSys.*;

@Autonomous(name="Right Auto", group="Autonomous")
public class RightAuto extends AutoBaseOpMode{

    Action preLoad;
    Action dropOff;
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
                .strafeTo(new Vector2d(4, -34.9), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();
        
        dropOff = drive.actionBuilder(new Pose2d(4, -34.9, Math.toRadians(270)))
//                .strafeTo(new Vector2d(25.4, -34.9), drive.maxVelConstraint, drive.defaultAccelConstraint)
//                .strafeTo(new Vector2d(46.4, -8.9), drive.maxVelConstraint, drive.defaultAccelConstraint)
//                .strafeTo(new Vector2d(46.2, -49.5), drive.maxVelConstraint, drive.maxAccelConstraint)
//                .strafeTo(new Vector2d(50.31, -13.74), drive.maxVelConstraint, drive.defaultAccelConstraint)
//                .strafeTo(new Vector2d(57.6, -14.5), drive.maxVelConstraint, drive.defaultAccelConstraint)
//                .strafeTo(new Vector2d(57.5, -49.5), drive.maxVelConstraint, drive.defaultAccelConstraint)
//                .strafeTo(new Vector2d(58.5, -14.5), drive.maxVelConstraint, drive.defaultAccelConstraint)
//                .strafeTo(new Vector2d(63.5, -14.6), drive.maxVelConstraint, drive.defaultAccelConstraint)
//                .strafeTo(new Vector2d(63.5, -62.5), drive.maxVelConstraint, drive.defaultAccelConstraint)
                .strafeTo(new Vector2d(11, -36))
                .splineToConstantHeading(new Vector2d(46.4, -8.9), Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .splineToConstantHeading(new Vector2d(46.2, -49.5 ),Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .splineToConstantHeading(new Vector2d(50.31, -13.74), Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .splineToConstantHeading(new Vector2d(57.6, -13.74), Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .splineToConstantHeading(new Vector2d(57.6, -49.5), Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .splineToConstantHeading(new Vector2d(58.5, -13.74), Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .splineToConstantHeading(new Vector2d(63.5, -13.74), Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .splineToConstantHeading(new Vector2d(63.5, -62.5), Math.toRadians(270), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();
        
        score1 = drive.actionBuilder(new Pose2d(63.5, -62.5, Math.toRadians(270)))
                .strafeTo(new Vector2d(6, -34.9), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();
        
        pickUp2 = drive.actionBuilder(new Pose2d(6, -34.9, Math.toRadians(270)))
                .strafeTo(new Vector2d(34.7, -39.8), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();
        
        score2 = drive.actionBuilder(new Pose2d(34.7, -39.8, Math.toRadians(270)))
                .strafeTo(new Vector2d(8, -34.9), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();
        
        pickUp3 = drive.actionBuilder(new Pose2d(8, -34.9, Math.toRadians(270)))
                .strafeTo(new Vector2d(34.7, -39.8), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();
        
        score3 = drive.actionBuilder(new Pose2d(34.7, -39.8, Math.toRadians(270)))
                .strafeTo(new Vector2d(10, -34.9), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();

        
        park = drive.actionBuilder(new Pose2d(10, -34.9, Math.toRadians(270)))
                .strafeTo(new Vector2d(16.7, -62.2), drive.maxVelConstraint, drive.maxAccelConstraint)
                .build();



        schedule(
            new SequentialCommandGroup(
                    // SCORE PRELOAD
                    new ParallelCommandGroup(
                            new ActionCommand(preLoad),
                            liftSys.goTo(LiftSys.HIGH_RUNG),
                            outtakeV4bSys.away()
                    ),
                    //outtake v4b sm shit
                    liftSys.goTo(LiftSys.HIGH_RUNG - 200),
                    outtakeClawSys.release(),
                    new WaitCommand(200),
                    //DROP OFF 1
                    new InstantCommand(()->drive.setTolerance(4, 0)),
                    new ParallelCommandGroup(
                            new ActionCommand(dropOff),
                            liftSys.goTo(LiftSys.NONE),
                            outtakeV4bSys.mid(),
                            intakeClawSys.release(),
                            outtakeClawSys.release()
                    ),
                    new InstantCommand(()->drive.setTolerance(1,4)),
                    intakeV4bSys.specimenIntake(),
                    //SCORE 1
                    new WaitCommand(100),
                    intakeClawSys.pinch(),
                    new WaitCommand(150),
                    intakeV4bSys.goToRoll(ROLL_IN_SPECIMEN),
                    new WaitCommand(150),
                    new ParallelCommandGroup(
                            extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
                            intakeV4bSys.goToPos(POS_SPECIMEN_IN),
                            outtakeV4bSys.specimen()
                    ),
                    new ParallelCommandGroup(
                            new ActionCommand(score1),
                            new SequentialCommandGroup(
                                    outtakeClawSys.grab(),
                                    intakeClawSys.release(),
                                    new WaitCommand(200),
                                    new ParallelCommandGroup(
                                            liftSys.goTo(LiftSys.HIGH_RUNG),
                                            outtakeV4bSys.away()
                                    )
                            )
                    ),
                    //outtake v4b some shit
                    liftSys.goTo(LiftSys.HIGH_RUNG-200),
                    new WaitCommand(200),
                    outtakeClawSys.release(),
                    new WaitCommand(100),
                    //PICKUP 2
                    new ParallelCommandGroup(
                            new ActionCommand(pickUp2),
                            liftSys.goTo(LiftSys.NONE),
                            new SequentialCommandGroup(
                                    new WaitCommand(500),
                                    outtakeV4bSys.mid()
                            )
                    ),
                    //SCORE 2
                    extendoSys.goTo(ExtendoSys.EXTENDO_MAX),
                    intakeV4bSys.specimenIntake(),
                    intakeClawSys.release(),
                    outtakeClawSys.release(),
                    new WaitCommand(300),
                    intakeClawSys.pinch(),
                    new WaitCommand(150),
                    intakeV4bSys.goToRoll(ROLL_IN_SPECIMEN),
                    extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
                    intakeV4bSys.goToPos(POS_SPECIMEN_IN),
                    outtakeV4bSys.specimen(),
                    new ParallelCommandGroup(
                            new ActionCommand(score2),
                            new SequentialCommandGroup(
                                    new WaitCommand(300),
                                    outtakeClawSys.grab(),
                                    intakeClawSys.release(),
                                    new WaitCommand(200),
                                    new ParallelCommandGroup(
                                            liftSys.goTo(LiftSys.HIGH_RUNG),
                                            outtakeV4bSys.away()
                                    )
                            )
                    ),
                    //score sm shit
                    liftSys.goTo(LiftSys.HIGH_RUNG - 200),
                    outtakeClawSys.release(),
                    new WaitCommand(50),
                    //PICKUP 3
                    new ParallelCommandGroup(
                            new ActionCommand(pickUp3),
                            liftSys.goTo(LiftSys.NONE),
                            new SequentialCommandGroup(
                                    new WaitCommand(500),
                                    outtakeV4bSys.mid()
                            )
                    ),
                    //SCORE 3
                    extendoSys.goTo(ExtendoSys.EXTENDO_MAX),
                    intakeV4bSys.specimenIntake(),
                    intakeClawSys.release(),
                    outtakeClawSys.release(),
                    new WaitCommand(300),
                    intakeClawSys.pinch(),
                    new WaitCommand(150),
                    intakeV4bSys.goToRoll(ROLL_IN_SPECIMEN),
                    extendoSys.goTo(ExtendoSys.EXTENDO_HOME),
                    intakeV4bSys.goToPos(POS_SPECIMEN_IN),
                    outtakeV4bSys.specimen(),
                    new ParallelCommandGroup(
                            new ActionCommand(score3),
                            new SequentialCommandGroup(
                                    new WaitCommand(300),
                                    outtakeClawSys.grab(),
                                    intakeClawSys.release(),
                                    new WaitCommand(200),
                                    new ParallelCommandGroup(
                                            liftSys.goTo(LiftSys.HIGH_RUNG),
                                            outtakeV4bSys.away()
                                    )
                            )
                    ),
                    //score sm shit
                    liftSys.goTo(LiftSys.HIGH_RUNG - 200),
                    outtakeClawSys.release(),
                    new WaitCommand(50),
                    //PARK
                    new ParallelCommandGroup(
                            new InstantCommand(()->drive.setTolerance(4, 0)),
                            new ActionCommand(park),
                            liftSys.goTo(LiftSys.NONE),
                            outtakeV4bSys.mid()
                    ),
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
