package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.ActionCommand;
import org.firstinspires.ftc.teamcode.util.math.Pose2D;

public class Robot {

    public static Pose2d startPose = new Pose2d(new Vector2d(0,0), Math.toRadians(90));

    public static void specimenScore(PinpointDrive drive, OuttakeV4BSys outtakeV4BSys, OuttakeClawSys outtakeClawSys, LiftSys liftSys) {
        Action score = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(8, -36), Math.toRadians(270))
                .build();

        schedule(
                new SequentialCommandGroup(
                        outtakeV4BSys.mid(),
                        new ParallelCommandGroup(
                            new ActionCommand(score),
                            liftSys.goTo(LiftSys.HIGH_RUNG)
                        ),
                        outtakeV4BSys.away(),
                        liftSys.goTo(LiftSys.HIGH_RUNG-400),
                        new WaitCommand(200),
                        outtakeClawSys.release()
                )
        );

    }

    public static void specimenPickup(PinpointDrive drive, OuttakeV4BSys outtakeV4BSys, OuttakeClawSys outtakeClawSys, LiftSys liftSys) {
        Action pickup = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(34.7, -58), Math.toRadians(90))
                .build();

        schedule(
                new ParallelCommandGroup(
                        new ActionCommand(pickup),
                        new SequentialCommandGroup(
                                liftSys.goTo(LiftSys.NONE),
                                outtakeV4BSys.specimen()
                        )
                ),
                new WaitCommand(200),
                outtakeClawSys.grab()
        );

    }

    public static void schedule(Command... cmd) {
        CommandScheduler.getInstance().schedule(cmd);
    }
}
