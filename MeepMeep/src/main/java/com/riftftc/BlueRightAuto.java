package com.riftftc;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BlueRightAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setDimensions(14, 17)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 6)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-16.6, 63, Math.toRadians(90)))
                        .lineTo(new Vector2d(-5.7, 36.1))
                        .waitSeconds(2)
                        .splineToConstantHeading(new Vector2d(-27.1, 32.09), Math.toRadians(270))
//                        .lineToSplineHeading(new Pose2d(-41.6, 61, Math.toRadians(270)))
//                        .waitSeconds(2)
//                        .lineTo(new Vector2d(-41.6, 62.3))
//                        .lineToSplineHeading(new Pose2d(-7.7, 36.1, Math.toRadians(90)))
//                        .waitSeconds(2)
//                        .lineToSplineHeading(new Pose2d(-41.6, 61, Math.toRadians(270)))
//                        .waitSeconds(2)
//                        .lineTo(new Vector2d(-41.6, 62.3))
//                        .lineToSplineHeading(new Pose2d(-9.7, 36.1, Math.toRadians(90)))
//                        .waitSeconds(2)
//                        .lineToSplineHeading(new Pose2d(-41.6, 61, Math.toRadians(270)))
//                        .waitSeconds(2)
//                        .lineTo(new Vector2d(-41.6, 62.3))
//                        .lineToSplineHeading(new Pose2d(-11.7, 36.1, Math.toRadians(90)))
//                        .waitSeconds(2)
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}