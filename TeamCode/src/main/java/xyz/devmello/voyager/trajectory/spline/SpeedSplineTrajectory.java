/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.trajectory.spline;

import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.math.Spline;
import xyz.devmello.voyager.trajectory.LinearTrajectory;

/**
 * A wrapper for {@link LinearTrajectory} that uses a spline to determine
 * the speed of the robot, allowing for very epic acceleration and deceleration.
 *
 * @author Colin Robertson
 * @since 0.6.1
 */
public class SpeedSplineTrajectory extends LinearTrajectory {
    private final Spline speedSpline;

    public SpeedSplineTrajectory(
        PointXYZ target,
        double tolerance,
        Angle angleTolerance,
        Spline speedSpline
    ) {
        super(target, 0.5, tolerance, angleTolerance);
        this.speedSpline = speedSpline;
    }

    @Override
    public double speed(PointXYZ current) {
        return speedSpline.interpolateY(current.x());
    }
}
