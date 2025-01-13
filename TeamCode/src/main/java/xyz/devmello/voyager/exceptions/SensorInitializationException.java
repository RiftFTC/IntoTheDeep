/*
 * Copyright (c) 2022.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.exceptions;

/**
 * An exception to be thrown whenever the initialization of a sensor causes
 * some sort of issue.
 *
 * @author Colin Robertson
 * @since 0.7.1
 */
public class SensorInitializationException extends RuntimeException {

    public SensorInitializationException(String message) {
        super(message);
    }
}
