/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.recording;

import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.time.Time;

/**
 * Used in conjunction with {@link MovementRecorder} to make Pathfinder
 * follow a set of pre-recorded motion snapshots.
 *
 * @author Colin Robertson
 * @since 0.6.1
 */
public class MovementPlayback {
    private final Voyager voyager;
    private MovementRecording recording;
    private boolean isPlaying = false;
    private int lastIndex = 0;
    private double lastSwitchMs = 0;

    /**
     * Create a new {@code MovementPlayback}.
     *
     * @param voyager the instance of Pathfinder that will be controlled
     *                   by this playback manager.
     */
    public MovementPlayback(Voyager voyager) {
        this.voyager = voyager;
    }

    /**
     * Start playing back a movement recording.
     *
     * @param recording the recording to play back.
     */
    public void startPlayback(MovementRecording recording) {
        lastIndex = 0;
        lastSwitchMs = 0;
        isPlaying = true;
        this.recording = recording;
    }

    /**
     * Stop the playback of a movement recording.
     */
    public void stopPlayback() {
        isPlaying = false;
    }

    public void tick() {
        if (!isPlaying) return;

        double currentMs = Time.ms();
        if (lastSwitchMs == 0) lastSwitchMs = currentMs;
        double elapsedMs = currentMs - lastSwitchMs;

        MovementRecord record = recording.getRecording().get(lastIndex);
        double requiredMs = record.getElapsedMs();
        if (elapsedMs >= requiredMs) {
            voyager.setTranslation(record.getTranslation());
            lastIndex += 1;
            lastSwitchMs = currentMs;
            if (lastIndex >= recording.getRecording().size()) stopPlayback();
        }
    }
}
