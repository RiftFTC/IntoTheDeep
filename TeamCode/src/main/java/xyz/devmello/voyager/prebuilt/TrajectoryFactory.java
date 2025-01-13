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

import java.util.ArrayList;
import java.util.List;
import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.trajectory.FastTrajectory;
import xyz.devmello.voyager.trajectory.LinearTrajectory;
import xyz.devmello.voyager.trajectory.Trajectory;

/**
 * Utility class used for working with trajectories. Instead of having to
 * use so many constructors, these static methods allow you to convert single
 * point (or multiple point) trajectory sequences, which can then be
 * used by Pathfinder.
 *
 * @author Colin Robertson
 * @since 0.1.0
 */
public class TrajectoryFactory {

    private TrajectoryFactory() {}

    public static List<Trajectory> getLinearTrajectories(
        List<PointXYZ> points,
        double speed,
        double tolerance,
        Angle angleTolerance
    ) {
        List<Trajectory> trajectories = new ArrayList<>(points.size() - 1);

        for (PointXYZ point : points) {
            LinearTrajectory trajectory = new LinearTrajectory(
                point,
                speed,
                tolerance,
                angleTolerance
            );

            trajectories.add(trajectory);
        }

        return trajectories;
    }

    public static List<Trajectory> getLinearTrajectories(
        List<PointXYZ> points,
        Voyager voyager
    ) {
        return getLinearTrajectories(
            points,
            voyager.getSpeed(),
            voyager.getTolerance(),
            voyager.getAngleTolerance()
        );
    }

    public static List<Trajectory> getFastTrajectories(
        PointXYZ start,
        List<PointXYZ> points,
        double speed
    ) {
        List<Trajectory> trajectories = new ArrayList<>(points.size() - 1);

        PointXYZ lastPoint = start;
        for (PointXYZ point : points) {
            FastTrajectory trajectory = new FastTrajectory(
                lastPoint,
                point,
                speed
            );

            trajectories.add(trajectory);

            lastPoint = point;
        }

        return trajectories;
    }

    public static List<Trajectory> getFastTrajectories(
        List<PointXYZ> points,
        Voyager voyager
    ) {
        PointXYZ position = voyager.getPosition();

        return getFastTrajectories(position, points, voyager.getSpeed());
    }

    public static Trajectory linearTrajectoryTo(
        PointXYZ target,
        double speed,
        double tolerance,
        Angle angleTolerance
    ) {
        return new LinearTrajectory(target, speed, tolerance, angleTolerance);
    }

    public static Trajectory linearTrajectoryTo(
        PointXYZ target,
        Voyager voyager
    ) {
        return linearTrajectoryTo(
            target,
            voyager.getSpeed(),
            voyager.getTolerance(),
            voyager.getAngleTolerance()
        );
    }

    public static Trajectory fastTrajectoryTo(
        PointXYZ start,
        PointXYZ target,
        double speed
    ) {
        return new FastTrajectory(start, target, speed);
    }
}
