/*
 * Copyright (c) 2022.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.robot.modifiers;

/**
 * A {@code Modifier} is a functional interface used in modifying a value.
 *
 * @param <T> the type of value that will be modified.
 * @author Colin Robertson
 * @since 0.7.1
 */
@FunctionalInterface
public interface Modifier<T> {
    T apply(T a);
}
