/*
 * Copyright (c) 2022.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.recording.recordables;

import xyz.devmello.voyager.recording.Recordable;

public class RecordableBoolean implements Recordable<Boolean> {
    private boolean value;

    public RecordableBoolean(boolean value) {
        this.value = value;
    }

    public RecordableBoolean() {
        this(false);
    }

    @Override
    public Boolean getRecordingValue() {
        return value;
    }

    @Override
    public void setRecordingValue(Object obj) {
        if (obj instanceof Boolean) {
            value = (Boolean) obj;
        }
    }
}
