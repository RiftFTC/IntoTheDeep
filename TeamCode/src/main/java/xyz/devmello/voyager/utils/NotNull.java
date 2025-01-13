/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.utils;

/**
 * Utility class used for verifying that objects are not null.
 *
 * @author Colin Robertson
 * @since 0.2.3
 */
public class NotNull {

    private NotNull() {}

    public static boolean isAnythingNull(Object... objects) {
        for (Object obj : objects) {
            if (obj == null) {
                return true;
            }
        }

        return false;
    }

    public static boolean isNothingNull(Object... objects) {
        return !isAnythingNull(objects);
    }

    public static void throwExceptionIfNull(String message, Object... objects) {
        if (isAnythingNull(objects)) {
            throw new NullPointerException(message);
        }
    }
}
