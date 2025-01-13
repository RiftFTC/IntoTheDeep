/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.prebuilt;

import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXY;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.trajectory.Trajectory;

/**
 * Helper class for utilizing {@link CircleSurround}.
 *
 * @author Colin Robertson
 * @see CircleSurround
 * @since 0.1.0
 */
public class CircleSurrounder {
    private final Voyager voyager;
    private final PointXY center;
    private final double radius;

    /**
     * Create a new {@code CircleSurrounder}.
     *
     * @param voyager the Pathfinder instance the surrounder should use.
     * @param center     the center of the circle.
     * @param radius     the radius of the circle.
     */
    public CircleSurrounder(
        Voyager voyager,
        PointXY center,
        double radius
    ) {
        this.voyager = voyager;
        this.center = center;
        this.radius = radius;
    }

    /**
     * Surround the circle. This will find the closest point, create a new
     * {@link xyz.devmello.voyager.trajectory.LinearTrajectory} to that
     * point, and then instruct Pathfinder to follow it.
     */
    public void surround() {
        PointXYZ robotPosition = voyager.getPosition();
        double speed = voyager.getSpeed();
        double tolerance = voyager.getTolerance();
        Angle angleTolerance = voyager.getAngleTolerance();

        Trajectory trajectory = CircleSurround.trajectoryToClosestPoint(
            robotPosition,
            center,
            radius,
            speed,
            tolerance,
            angleTolerance
        );

        voyager.followTrajectory(trajectory);
    }

    /**
     * Surround the circle. This will find the closest point, create a new
     * {@link xyz.devmello.voyager.trajectory.FastTrajectory} to that
     * point, and then instruct Pathfinder to follow it.
     */
    public void fastSurround() {
        PointXYZ robotPosition = voyager.getPosition();
        double speed = voyager.getSpeed();
        double tolerance = voyager.getTolerance();
        Angle angleTolerance = voyager.getAngleTolerance();

        Trajectory trajectory = CircleSurround.fastTrajectoryToClosestPoint(
            robotPosition,
            center,
            radius,
            speed
        );

        voyager.followTrajectory(trajectory);
    }
}
