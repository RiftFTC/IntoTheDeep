package com.riftftc;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;
import java.util.Arrays;

public class MeepMeepTesting {

    public static class Params {
        // IMU orientation
        // drive model parameters
        public double inPerTick = 1;
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 11.594258057297543;

        // feedforward parameters (in tick units)
        public double kS = 1.7586345497107194;
        public double kV = 0.1424830984486189;
        public double kA = 0.0001;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 7;
        public double lateralGain = 11;
        public double headingGain = 7; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    public static final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public static final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public static final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 11.594258057297543)
                .setDimensions(14,17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(16.7, -62.2, Math.toRadians(270)))
                //PRELOAD
                .strafeTo(new Vector2d(4, -34.9))
                                .waitSeconds(1)
                //DROP OFF ALL 3
                .strafeToLinearHeading(new Vector2d(11, -36), Math.toRadians(270),defaultVelConstraint, defaultAccelConstraint)
                .splineToConstantHeading(new Vector2d(46.4, -14), Math.toRadians(270), defaultVelConstraint, defaultAccelConstraint)
                .splineToConstantHeading(new Vector2d(46.2, -49.5 ),Math.toRadians(270), defaultVelConstraint, defaultAccelConstraint)
                .splineToConstantHeading(new Vector2d(50.31, -13.74), Math.toRadians(270), defaultVelConstraint, defaultAccelConstraint)
                .splineToConstantHeading(new Vector2d(57.6, -13.74), Math.toRadians(270), defaultVelConstraint, defaultAccelConstraint)
                .splineToConstantHeading(new Vector2d(57.6, -49.5), Math.toRadians(270), defaultVelConstraint, defaultAccelConstraint)
                .splineToConstantHeading(new Vector2d(58.5, -13.74), Math.toRadians(270), defaultVelConstraint, defaultAccelConstraint)
                .splineToConstantHeading(new Vector2d(63.5, -13.74), Math.toRadians(270), defaultVelConstraint, defaultAccelConstraint)
                .splineToConstantHeading(new Vector2d(63.5, -52), Math.toRadians(270), defaultVelConstraint, defaultAccelConstraint)
                // SCORE 1
                        .strafeToLinearHeading(new Vector2d(34.7, -60), Math.toRadians(90), defaultVelConstraint, defaultAccelConstraint)
                        .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(6, -34.9), Math.toRadians(270),defaultVelConstraint, defaultAccelConstraint)
                .waitSeconds(1)
//                // PICKUP 2
//                .strafeTo(new Vector2d(34.7, -39.8), defaultVelConstraint, defaultAccelConstraint)
//                // SCORE 2
//                .strafeTo(new Vector2d(8, -34.9), defaultVelConstraint, defaultAccelConstraint)
//                // PICKUP 3
//                .strafeTo(new Vector2d(34.7, -39.8), defaultVelConstraint, defaultAccelConstraint)
//                // SCORE 3
//                .strafeTo(new Vector2d(10, -34.9), defaultVelConstraint, defaultAccelConstraint)
                .strafeToLinearHeading(new Vector2d(34.7, -60), Math.toRadians(90),defaultVelConstraint, defaultAccelConstraint)
                                .waitSeconds(0.5)
                                .strafeToLinearHeading(new Vector2d(8, -34.9), Math.toRadians(270),defaultVelConstraint, defaultAccelConstraint)
                                .waitSeconds(1)
                                .strafeToLinearHeading(new Vector2d(34.7, -60), Math.toRadians(90),defaultVelConstraint, defaultAccelConstraint)
                                .waitSeconds(0.5)
                                .strafeToLinearHeading(new Vector2d(10, -34.9), Math.toRadians(270),defaultVelConstraint, defaultAccelConstraint)
                                .waitSeconds(1)




                // PARK
                .strafeToLinearHeading(new Vector2d(23.3, -46.9), Math.toRadians(315),defaultVelConstraint, defaultAccelConstraint)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}