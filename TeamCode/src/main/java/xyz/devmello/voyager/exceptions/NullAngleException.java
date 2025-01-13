/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.exceptions;

import xyz.devmello.voyager.geometry.Angle;

/**
 * Exception thrown whenever an operation is requested to be performed on
 * an angle that is... you guessed it... null!
 *
 * @author Colin Robertson
 * @since 0.0.0
 */
public class NullAngleException extends RuntimeException {

    public NullAngleException(String message) {
        super(message);
    }

    public static void throwIfInvalid(String message, Angle angle) {
        if (angle == null) {
            throw new NullAngleException(message);
        }
    }
}
