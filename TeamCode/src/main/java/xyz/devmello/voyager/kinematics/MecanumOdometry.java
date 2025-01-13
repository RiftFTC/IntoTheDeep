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
import xyz.devmello.voyager.geometry.PointXYZ;

/**
 * Odometry for a mecanum chassis.
 *
 * @author Colin Robertson
 * @since 0.5.0
 */
public class MecanumOdometry extends GenericOdometry<MecanumState> {

    public MecanumOdometry(
        Kinematics<MecanumState> kinematics,
        Angle gyroAngle,
        PointXYZ initialPosition
    ) {
        super(kinematics, gyroAngle, initialPosition);
    }
}
