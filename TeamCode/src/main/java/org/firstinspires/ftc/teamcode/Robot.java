package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.teamcode.opencv.SampleTrackPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.subsystem.*;
import org.firstinspires.ftc.teamcode.util.ActionCommand;
import org.firstinspires.ftc.teamcode.util.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.math.Precision;
import org.opencv.core.Point;
import xyz.devmello.voyager.robot.Drive;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeV4bSys.POS_DOWN;

public class Robot {

    public static Pose2d startPose = new Pose2d(new Vector2d(0,0), Math.toRadians(90));

    public static void specimenScore(PinpointDrive drive, OuttakeV4BSys outtakeV4BSys, OuttakeClawSys outtakeClawSys, LiftSys liftSys) {
        DriveSys.AUTOMATION = true;
        drive.setBrake();
        Action score = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(8, -37.1), Math.toRadians(270))
                .build();

        schedule(
                new SequentialCommandGroup(
                        outtakeClawSys.grab(),
                        new WaitCommand(300),
                        new SequentialCommandGroup(
                                outtakeV4BSys.mid(),
                                new ParallelCommandGroup(
                                        new ActionCommand(score),
                                        liftSys.goTo(LiftSys.HIGH_RUNG)
                                ),
                                outtakeV4BSys.specimenScore(),
                                liftSys.goTo(LiftSys.HIGH_RUNG-400),
                                new WaitCommand(200),
                                outtakeClawSys.release(),
                                new InstantCommand(drive::setCoast),
                                new InstantCommand(()->DriveSys.AUTOMATION = false)
                        )
                )
        );
    }

    public static void specimenPickup(PinpointDrive drive, OuttakeV4BSys outtakeV4BSys, OuttakeClawSys outtakeClawSys, LiftSys liftSys) {
        DriveSys.AUTOMATION = true;
        drive.setBrake();
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
                new InstantCommand(drive::setCoast),
                new InstantCommand(()->DriveSys.AUTOMATION = false)
        );

    }

    public static void samplePickup(PinpointDrive drive, OuttakeV4BSys outtakeV4BSys, OuttakeClawSys outtakeClawSys, LiftSys liftSys, ExtendoSys extendoSys, IntakeV4bSys intakeV4bSys, IntakeClawSys intakeClawSys, SampleTrackPipeline pipeline) {
        IntakeClawSys.AUTO = true;
        DriveSys.AUTOMATION = true;
        drive.setBrake();
        double xCoverageInches = 14.80314960629921;
        double yCoverageInches = 10.7440945;
        double cameraOffsetInches = 1;

        ArrayList<SampleTrackPipeline.AnalyzedStone> clientStoneList = pipeline.getDetectedStones();

        if (clientStoneList.isEmpty()) {
            return;
        }

        // Assume the frame size is known
        Point frameCenter = new Point(pipeline.frameSize.width / 2, pipeline.frameSize.height / 2);
        SampleTrackPipeline.AnalyzedStone closestStone = null;
        double closestDistance = Double.MAX_VALUE;

        for (SampleTrackPipeline.AnalyzedStone stone : clientStoneList) {
            double distanceToCenter = Math.hypot(stone.center.x - frameCenter.x, stone.center.y - frameCenter.y);
            if (distanceToCenter < closestDistance) {
                closestDistance = distanceToCenter;
                closestStone = stone;
            }
        }

        if (closestStone == null) {
            return;
        }

        // Calculate the offset in pixels from the frame center
        double deltaX = closestStone.center.x - frameCenter.x;
        double deltaY = closestStone.center.y - frameCenter.y;

        // Convert pixel offset to real-world offset (in inches)
        double realWorldX = (deltaX / pipeline.frameSize.width) * xCoverageInches - 2;
        double realWorldY = (deltaY / pipeline.frameSize.height) * yCoverageInches - cameraOffsetInches;

        // Log the results
        Log.i("goTo X", String.valueOf(realWorldX));
        Log.i("goTo Y", String.valueOf(realWorldY));

        SampleTrackPipeline.GoToStone sample = new SampleTrackPipeline.GoToStone(
                new Vector2d(drive.pose.position.x + realWorldX, drive.pose.position.y + realWorldY),
                closestStone.angle
        );

        Action goTo = drive.actionBuilder(drive.pose).strafeTo(sample.position).build();
        double angle = Math.round(Precision.calculateWeightedValue(IntakeClawSys.YAW_LEFT, IntakeClawSys.YAW_RIGHT, (sample.angle % 179) / 180) * 5) / 5.0;

        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ActionCommand(goTo),
                                intakeClawSys.rotateYaw(angle)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                intakeV4bSys.goToPos(POS_DOWN - 0.03),
                                new WaitCommand(200),
                                intakeClawSys.pinch(),
                                new WaitCommand(200),
                                intakeV4bSys.dropOff(),
                                extendoSys.goTo(ExtendoSys.EXTENDO_HOME)
                        )
                        //implement a transfer to the outtake
                ),
                new InstantCommand(drive::setCoast),
                new InstantCommand(()->DriveSys.AUTOMATION = false)
        );

        IntakeClawSys.AUTO = false;
    }

    public static boolean isNear(Pose2d pose, Pose2d target, double tolerance) {
        return Math.abs(pose.position.x - target.position.x) < tolerance && Math.abs(pose.position.y - target.position.y) < tolerance;
    }

    public static void schedule(Command... cmd) {
        CommandScheduler.getInstance().schedule(cmd);
    }


}
