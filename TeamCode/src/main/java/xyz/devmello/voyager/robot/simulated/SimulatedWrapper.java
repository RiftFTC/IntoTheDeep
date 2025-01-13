/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.robot.simulated;

import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.robot.Robot;

/**
 * A wrapper for instances of {@link SimulatedDrive} and
 * {@link SimulatedOdometry}.
 *
 * @author Colin Robertson
 * @since 0.15.0
 */
public class SimulatedWrapper {
    private final SimulatedDrive drive;
    private final SimulatedOdometry odometry;

    public SimulatedWrapper(SimulatedDrive drive, SimulatedOdometry odometry) {
        this.drive = drive;
        this.odometry = odometry;

        this.drive.setDriveModifier(
                translation -> {
                    PointXYZ pos = odometry.getPosition();

                    odometry.setRawPosition(pos.applyTranslation(translation));

                    return translation;
                }
            );
    }

    public SimulatedDrive getDrive() {
        return drive;
    }

    public SimulatedOdometry getOdometry() {
        return odometry;
    }

    public Robot getRobot() {
        return new Robot(drive, odometry);
    }
}
