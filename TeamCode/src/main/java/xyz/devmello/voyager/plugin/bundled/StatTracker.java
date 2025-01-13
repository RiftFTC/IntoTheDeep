/*
 * Copyright (c) 2022.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.plugin.bundled;

import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.follower.Follower;
import xyz.devmello.voyager.geometry.PointXY;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.math.RollingAverage;
import xyz.devmello.voyager.plugin.PathfinderPlugin;
import xyz.devmello.voyager.time.Time;

/**
 * Rudimentary plugin that tracks statistics on Pathfinder usage. This is
 * mostly used for debugging and profiling purposes - one of the key
 * measurements this plugin provides is the "ticks per second" rate of
 * Pathfinder. A higher tick rate means Pathfinder is running smoothly.
 * A lower tick rate means the opposite.
 *
 * @author Colin Robertson
 * @since 0.10.3
 */
public class StatTracker extends PathfinderPlugin {
    public static final String KEY_TPS = "pf_tps";
    public static final String KEY_TICKS = "pf_ticks";
    private static final String NAME = "StatTracker";
    public static double SECOND_MS_DURATION = 1_000;
    private final RollingAverage ticksPerSecond = new RollingAverage(10);
    private long ticks = 0;
    private long followersFinished = 0;
    private double totalDistance = 0;
    private PointXY lastPoint = null;
    private double lastMs = 0;

    @Override
    public String getName() {
        return NAME;
    }

    @Override
    public void onTick(Voyager voyager) {
        if (lastMs == 0) lastMs = Time.ms();

        ticks++;

        PointXYZ position = voyager.getPosition();

        if (lastPoint == null) lastPoint = position;

        totalDistance += lastPoint.absDistance(position);

        double currentMs = Time.ms();
        double elapsedSeconds = (currentMs - lastMs) / SECOND_MS_DURATION;
        if (elapsedSeconds < 1 / SECOND_MS_DURATION) return;
        lastMs = currentMs;
        double tps = 1 / elapsedSeconds;
        ticksPerSecond.add(tps);

        voyager.putData(KEY_TPS, ticksPerSecond.average());
        voyager.putData(KEY_TICKS, ticks);
    }

    @Override
    public void onFinishFollower(Voyager voyager, Follower follower) {
        followersFinished++;
    }
}
