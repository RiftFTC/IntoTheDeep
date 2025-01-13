/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.robot;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import xyz.devmello.voyager.geometry.Translation;

/**
 * Abstract implementation of the {@link Drive} interface. This class handles
 * the {@link #getDriveModifier()} and {@link #setDriveModifier(Function)} methods
 * required by the {@code Modifiable} interface.
 *
 * <p>
 * ALTHOUGH THIS IS NOT DEPRECATED, I WOULD STRONGLY ENCOURAGE YOU TO USE
 * {@link ImprovedAbstractDrive} INSTEAD.
 * </p>
 *
 * @author Colin Robertson
 * @since 0.0.0
 * @see ImprovedAbstractDrive
 */
public class AbstractDrive implements Drive {
    private final Consumer<Translation> consumer;
    private final Supplier<Translation> supplier;
    private Function<Translation, Translation> modifier = t -> t;

    /**
     * Create a new {@code AbstractDrive}.
     *
     * @param consumer a consumer for the translation. This should accept
     *                 outputted translations. All translations will be
     *                 modified using the {@link #modifier}.
     * @param supplier a supplier for the translation. This method should
     *                 return whatever translation was last set to the robot.
     */
    public AbstractDrive(
        Consumer<Translation> consumer,
        Supplier<Translation> supplier
    ) {
        this.consumer = consumer;
        this.supplier = supplier;
    }

    @Override
    public Translation getTranslation() {
        return supplier.get();
    }

    @Override
    public void setTranslation(Translation translation) {
        consumer.accept(modifier.apply(translation));
    }

    @Override
    public Function<Translation, Translation> getDriveModifier() {
        return modifier;
    }

    @Override
    public void setDriveModifier(Function<Translation, Translation> modifier) {
        this.modifier = modifier;
    }
}
