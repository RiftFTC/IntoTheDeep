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
import xyz.devmello.voyager.follower.Follower;
import xyz.devmello.voyager.follower.FollowerGenerator;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.robot.Robot;
import xyz.devmello.voyager.trajectory.FastTrajectory;
import xyz.devmello.voyager.trajectory.LinearTrajectory;
import xyz.devmello.voyager.trajectory.Trajectory;

/**
 * Utility class for building a series of points.
 *
 * @author Colin Robertson
 * @since 0.1.0
 */
public class PathBuilder {
    private final List<PointXYZ> targets;

    public PathBuilder() {
        this(10);
    }

    public PathBuilder(int initialSize) {
        targets = new ArrayList<>(initialSize);
    }

    public static List<Follower> followersFromTrajectories(
        Robot robot,
        FollowerGenerator followerGenerator,
        List<Trajectory> trajectories
    ) {
        List<Follower> followers = new ArrayList<>(trajectories.size());

        for (Trajectory trajectory : trajectories) {
            followers.add(followerGenerator.generate(robot, trajectory));
        }

        return followers;
    }

    public List<PointXYZ> getTargets() {
        return targets;
    }

    public List<Trajectory> linearPath(
        double speed,
        double tolerance,
        Angle angleTolerance
    ) {
        List<Trajectory> trajectories = new ArrayList<>(targets.size());

        for (PointXYZ target : targets) {
            trajectories.add(
                new LinearTrajectory(target, speed, tolerance, angleTolerance)
            );
        }

        return trajectories;
    }

    public List<Trajectory> fastPath(PointXYZ start, double speed) {
        List<Trajectory> trajectories = new ArrayList<>(targets.size());
        PointXYZ previousPoint = start;

        for (PointXYZ target : targets) {
            trajectories.add(new FastTrajectory(previousPoint, target, speed));

            previousPoint = target;
        }

        return trajectories;
    }

    public List<Follower> linearPathFollowers(
        Robot robot,
        FollowerGenerator followerGenerator,
        double speed,
        double tolerance,
        Angle angleTolerance
    ) {
        List<Trajectory> trajectories = linearPath(
            speed,
            tolerance,
            angleTolerance
        );

        return followersFromTrajectories(
            robot,
            followerGenerator,
            trajectories
        );
    }

    public List<Follower> fastPathFollowers(
        Robot robot,
        FollowerGenerator followerGenerator,
        double speed
    ) {
        List<Trajectory> trajectories = fastPath(
            robot.odometry().getPosition(),
            speed
        );

        return followersFromTrajectories(
            robot,
            followerGenerator,
            trajectories
        );
    }

    public void addTarget(PointXYZ target) {
        targets.add(target);
    }

    public void addTarget(Angle angleToMoveAt, double distanceToMove) {
        addTarget(
            targets
                .get(targets.size() - 1)
                .inDirection(distanceToMove, angleToMoveAt)
        );
    }
}
