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
import xyz.devmello.voyager.geometry.PointXY;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.math.Spline;
import xyz.devmello.voyager.trajectory.Trajectory;

/**
 * A trajectory based on two splines - firstly, a spline representing
 * the target position of the robot, and additionally, a spline representing
 * the angle of the robot. So cool! So epic! So true!
 *
 * @author Colin Robertson
 * @since 0.6.1
 */
public class AngleSplineTrajectory implements Trajectory {
    private final Spline spline;
    private final AngleSpline angleSpline;
    private final double speed;
    private final double step;
    private final double tolerance;

    public AngleSplineTrajectory(
        Spline spline,
        AngleSpline angleSpline,
        double speed,
        double step,
        double tolerance
    ) {
        this.spline = spline;
        this.angleSpline = angleSpline;
        this.speed = speed;
        this.step = step;
        this.tolerance = tolerance;
    }

    @Override
    public PointXYZ nextMarker(PointXYZ current) {
        double x = current.x() + step;
        PointXY interpolatedPoint = spline.interpolate(x);
        Angle interpolatedAngle = angleSpline.getAngleTarget(x);
        return interpolatedPoint.withHeading(interpolatedAngle);
    }

    @Override
    public boolean isDone(PointXYZ current) {
        return current.isNear(spline.getEndPoint(), tolerance);
    }

    @Override
    public double speed(PointXYZ current) {
        return speed;
    }
}
