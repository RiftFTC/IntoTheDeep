/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.robot.modifiers;

import java.util.function.Function;
import xyz.devmello.voyager.geometry.Translation;

/**
 * Simple modifier for drivetrains.
 *
 * @author Colin Robertson
 * @since 0.0.0
 */
public class DriveModifier {
    public static Function<Translation, Translation> SWAP_XY =
        DriveModifier::swapXY;
    public static Function<Translation, Translation> INVERT_X =
        DriveModifier::invertX;
    public static Function<Translation, Translation> INVERT_Y =
        DriveModifier::invertY;
    public static Function<Translation, Translation> INVERT_Z =
        DriveModifier::invertZ;

    /**
     * Should X and Y be swapped?
     */
    private boolean swapXY;

    /**
     * Should X be inverted (multiplied by -1)?
     */
    private boolean invertX;

    /**
     * Should Y be inverted (multiplied by -1)?
     */
    private boolean invertY;

    public static Translation swapXY(Translation translation) {
        return new Translation(
            translation.vy(),
            translation.vz(),
            translation.vz()
        );
    }

    public static Translation invertX(Translation translation) {
        return translation.multiply(-1, 1, 1);
    }

    public static Translation invertY(Translation translation) {
        return translation.multiply(1, -1, 1);
    }

    public static Translation invertZ(Translation translation) {
        return translation.multiply(1, 1, -1);
    }

    /**
     * Should X and Y be swapped?
     *
     * @param swapXY should X and Y be swapped?
     * @return this.
     */
    public DriveModifier swapXY(boolean swapXY) {
        this.swapXY = swapXY;
        return this;
    }

    /**
     * Should X be inverted (multiplied by -1)?
     *
     * @param invertX should X be inverted?
     * @return this.
     */
    public DriveModifier invertX(boolean invertX) {
        this.invertX = invertX;
        return this;
    }

    /**
     * Should Y be inverted (multiplied by -1)?
     *
     * @param invertY should Y be inverted?
     * @return this.
     */
    @SuppressWarnings("UnusedReturnValue")
    public DriveModifier invertY(boolean invertY) {
        this.invertY = invertY;
        return this;
    }

    /**
     * Modify things! Awesome!
     *
     * @param translation the translation to modify.
     * @return the modified translation.
     */
    public Translation modify(Translation translation) {
        double x = swapXY ? translation.vy() : translation.vx();
        double y = swapXY ? translation.vx() : translation.vy();
        x = invertX ? x * -1 : x;
        y = invertY ? y * -1 : y;

        return new Translation(x, y, translation.vz());
    }
}
