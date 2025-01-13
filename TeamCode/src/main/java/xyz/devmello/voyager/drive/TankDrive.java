/*
 * Copyright (c) 2022.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.drive;

import xyz.devmello.voyager.geometry.Translation;
import xyz.devmello.voyager.kinematics.Kinematics;
import xyz.devmello.voyager.kinematics.TankKinematics;
import xyz.devmello.voyager.kinematics.TankState;
import xyz.devmello.voyager.robot.ImprovedAbstractDrive;
import xyz.devmello.voyager.robot.components.Motor;

/**
 * Tank drive implementation. Makes use of {@link TankKinematics}.
 *
 * @author Colin Robertson
 * @since 2.1.1
 */
public class TankDrive extends ImprovedAbstractDrive {
    private final Motor right;
    private final Motor left;
    private final Kinematics<TankState> kinematics;

    public TankDrive(
        Motor right,
        Motor left,
        Kinematics<TankState> kinematics
    ) {
        if (right == null) throw new NullPointerException(
            "Cannot create a TankDrive with a null right motor!"
        );

        if (left == null) throw new NullPointerException(
            "Cannot create a TankDrive with a null left motor!"
        );

        if (kinematics == null) throw new NullPointerException(
            "Cannot create a TankDrive with null kinematics!"
        );

        this.right = right;
        this.left = left;
        this.kinematics = kinematics;
    }

    public TankDrive(
        Motor right,
        Motor left,
        double turnCoefficient,
        double trackWidth
    ) {
        this(right, left, new TankKinematics(turnCoefficient, trackWidth));
    }

    @Override
    public void abstractSetTranslation(Translation translation) {
        TankState state = kinematics.calculate(translation);

        right.setPower(state.right());
        left.setPower(state.left());
    }
}
