/*
 * Copyright (c) 2022.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.listening;

import java.util.function.Predicate;
import java.util.function.Supplier;
import xyz.devmello.voyager.Voyager;

/**
 * A listener that accepts {@code Pathfinder} as a parameter.
 *
 * @author Colin Robertson
 * @since 0.7.1
 * @deprecated this is some pretty bad code and I wouldn't recommend you make
 * use of it...
 */
@Deprecated
public class PathfinderListener implements Supplier<Boolean> {
    private final Voyager voyager;
    private final Predicate<Voyager> predicate;

    /**
     * Create a new {@code PathfinderListener}.
     *
     * @param voyager the {@code Pathfinder} instance that's being operated
     *                   on.
     * @param predicate  a predicate that determines whether this supplier
     *                   should return true.
     */
    public PathfinderListener(
        Voyager voyager,
        Predicate<Voyager> predicate
    ) {
        this.voyager = voyager;
        this.predicate = predicate;
    }

    @Override
    public Boolean get() {
        return predicate.test(voyager);
    }
}
