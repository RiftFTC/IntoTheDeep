/*
 * Copyright (c) 2022.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.robot.sensors;

import xyz.devmello.voyager.units.Conversions;

public abstract class InchesDistanceSensor implements DistanceSensor {

    @Override
    public double getDistanceCentimeters() {
        return Conversions.inchesToCm(getDistanceInches());
    }

    @Override
    public double getDistanceMeters() {
        return Conversions.inchesToM(getDistanceInches());
    }
}
