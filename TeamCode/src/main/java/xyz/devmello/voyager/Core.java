/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager;

import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.Translation;
import xyz.devmello.voyager.trajectory.LinearTrajectory;

/**
 * Various pseudo-constant values used throughout Pathfinder's core module.
 * These values are used as constants in several classes to allow complete
 * customizability of the inner workings of Pathfinder.
 *
 * @author Colin Robertson
 * @since 1.1.0
 */
public class Core {
    /**
     * The default speed value for new instances of {@code Pathfinder}.
     */
    public static final double pathfinderDefaultSpeed = -1;

    /**
     * The default tolerance value for new instances of {@code Pathfinder}.
     */
    public static final double pathfinderDefaultTolerance = -1;

    /**
     * The default angle tolerance value for new instances of
     * {@code Pathfinder}.
     */
    public static final Angle pathfinderDefaultAngleTolerance = null;

    /**
     * The default timeout value for new instances of {@code Pathfinder}.
     */
    public static final double pathfinderDefaultTimeout = Double.MAX_VALUE;

    /**
     * The default is minimal value for new instances of {@code Pathfinder}.
     */
    public static final boolean pathfinderDefaultIsMinimal = false;

    /**
     * The default random string length for new instances of
     * {@code Pathfinder}.
     */
    public static final int pathfinderRandomStringLength = 10;

    /**
     * The default wait sleep time value (in milliseconds) for new instances
     * of {@code Pathfinder}.
     */
    public static final int pathfinderWaitSleepTimeMs = 10;

    /**
     * The default coefficient for the
     * {@link Voyager#splineTo(xyz.devmello.voyager.geometry.PointXYZ...)}
     * method.
     */
    public static final double pathfinderSplineStepCoefficient = 10;

    /**
     * The default coefficient for the
     * {@link Voyager#splineTo(xyz.devmello.voyager.geometry.PointXYZ...)}
     * method.
     */
    public static final double pathfinderStepDivisor = 20;

    /**
     * The default speed value for new instances of {@code Pathfinder}.
     */
    public static final Translation pathfinderDefaultTranslation =
        Translation.ZERO;

    /**
     * The minimum delay (in milliseconds) between snapshots when using
     * the movement recorder.
     */
    public static final double movementRecorderMinDelayMs = 25;

    /**
     * The default tolerance value for determining if two
     * {@link LinearTrajectory} instances are equal.
     */
    public static final double linearTrajectoryTolerance = 0.01;

    /**
     * The default format for a trajectory.
     */
    public static final String linearTrajectoryFormat =
        "Linear trajectory to %s at %s speed (tolerance %s %s)";

    /**
     * Format for {@code FastTrajectory}'s {@code toString()} method.
     */
    public static final String fastTrajectoryFormat =
        "Fast trajectory from %s to %s at %s speed";

    /**
     * Format for {@code AdvancedSplineTrajectory}'s {@code toString()} method.
     */
    public static final String advancedSplineTrajectoryFormat =
        "Advanced spline trajectory following spline %s (tolerance %s %s)";

    /**
     * Format for {@code CommandTaskTrajectory}'s {@code toString()} method.
     */
    public static final String commandTaskTrajectoryFormat =
        "CommandTaskTrajectory";

    /**
     * Format for {@code TimedTrajectory}'s {@code toString()} method.
     */
    public static final String timedTrajectoryFormat =
        "Timed trajectory (translation: <%s> time: <%s ms>)";

    /**
     * The multiplier that will be applied to the trajectory's speed
     * whenever the {@code isDone(PointXYZ)} method return trues.
     */
    public static final double linearTrajectoryIsDoneSpeedMultiplier = 0;

    /**
     * Tolerance used in determining if two {@code AdvancedSplineTrajectory}s
     * are the same.
     */
    public static final double advancedSplineTrajectoryTolerance = 0.01;

    /**
     * Tolerance used in determining if two {@code AdvancedSplineTrajectory}s
     * are the same.
     */
    public static final double advancedSplineTrajectoryDuplicateOffset = 0.01;

    /**
     * The default priority for {@code ListenerBuilder} instances.
     */
    public static final int listenerBuilderDefaultPriority = 0;

    /**
     * The default expiration for {@code ListenerBuilder} instances.
     */
    public static final double listenerBuilderDefaultExpiration =
        Double.MAX_VALUE;

    /**
     * The default maximum executions for {@code ListenerBuilder} instances.
     */
    public static final int listenerBuilderDefaultMaximumExecutions =
        Integer.MAX_VALUE;

    /**
     * The default cooldown (ms) for {@code ListenerBuilder} instances.
     */
    public static final int listenerBuilderDefaultCooldownMs = 0;

    /**
     * The length of random strings for automatically-generated keys for
     * {@code ListenerManager}.
     */
    public static final int listenerManagerRandomStringLength = 10;

    private Core() {}
}
