/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.kinematics;

import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.Translation;
import xyz.devmello.voyager.math.MinMax;

/**
 * The most simple implementation of mecanum kinematics. For most use
 * cases, this should suffice. If you need more advanced mecanum kinematics,
 * this class isn't for you.
 *
 * @author Colin Robertson
 * @since 0.0.0
 */
public class RelativeMecanumKinematics implements Kinematics<MecanumState> {
    /**
     * The angles of each of the wheels.
     */
    private static final Angle[] WHEEL_ANGLES = new Angle[] {
        Angle.DEG_45, // FRONT LEFT
        Angle.DEG_315, // FRONT RIGHT
        Angle.DEG_315, // BACK LEFT
        Angle.DEG_45 // BACK RIGHT
    };

    /**
     * The kinematics' minimum magnitude.
     */
    private final double minMagnitude;

    /**
     * The kinematics' maximum magnitude.
     */
    private final double maxMagnitude;

    /**
     * The kinematics' turn magnitude.
     */
    private final double turnMagnitude;

    /**
     * The kinematics' angle offset.
     */
    private final Angle angleOffset;

    /**
     * Create a new instance of the {@code RelativeMecanumKinematics} class.
     *
     * @param minMagnitude the minimum magnitude.
     * @param maxMagnitude the maximum magnitude.
     * @param angleOffset  the angle offset for the kinematics.
     */
    public RelativeMecanumKinematics(
        double minMagnitude,
        double maxMagnitude,
        Angle angleOffset
    ) {
        this(minMagnitude, maxMagnitude, 1.0, angleOffset);
    }

    /**
     * Create a new instance of the {@code RelativeMecanumKinematics} class.
     *
     * @param minMagnitude  the minimum magnitude.
     * @param maxMagnitude  the maximum magnitude.
     * @param turnMagnitude the turn magnitude.
     * @param angleOffset   the angle offset for the kinematics.
     */
    public RelativeMecanumKinematics(
        double minMagnitude,
        double maxMagnitude,
        double turnMagnitude,
        Angle angleOffset
    ) {
        this.minMagnitude = minMagnitude;
        this.maxMagnitude = maxMagnitude;
        this.turnMagnitude = turnMagnitude;
        this.angleOffset = angleOffset;
    }

    private static double calculatePower(Angle movement, Angle wheel) {
        double x = movement.sin() * wheel.sin();
        double y = movement.cos() * wheel.cos();

        return x + y;
    }

    @Override
    public MecanumState calculate(Translation translation) {
        double xyMagnitude = MinMax.clip(
            Math.hypot(translation.vx(), translation.vy()),
            minMagnitude,
            maxMagnitude
        );
        Angle angle = Angle
            .atan2(translation.vy(), translation.vx())
            .add(angleOffset);
        double turn = translation.vz() * turnMagnitude;

        double fl =
            (calculatePower(angle, WHEEL_ANGLES[0]) * xyMagnitude) + turn;
        double fr =
            (calculatePower(angle, WHEEL_ANGLES[1]) * xyMagnitude) - turn;
        double bl =
            (calculatePower(angle, WHEEL_ANGLES[2]) * xyMagnitude) + turn;
        double br =
            (calculatePower(angle, WHEEL_ANGLES[3]) * xyMagnitude) - turn;

        return new MecanumState(fl, fr, bl, br).normalizeFromMaxUnderOne();
    }

    @Override
    public Translation toTranslation(MecanumState state) {
        throw new RuntimeException(
            "Cannot convert a mecanum state to a translation " +
            "using the relative mecanum kinematics."
        );
    }
}
