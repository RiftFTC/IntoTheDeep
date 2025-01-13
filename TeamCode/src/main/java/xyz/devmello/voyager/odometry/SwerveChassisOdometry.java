/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.odometry;

import java.util.function.Supplier;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.kinematics.Kinematics;
import xyz.devmello.voyager.kinematics.SwerveDriveOdometry;
import xyz.devmello.voyager.kinematics.SwerveModuleState;
import xyz.devmello.voyager.kinematics.SwerveState;
import xyz.devmello.voyager.robot.AbstractOdometry;
import xyz.devmello.voyager.time.Time;

public class SwerveChassisOdometry extends AbstractOdometry {
    private final SwerveModuleOdometry frontRightOdometry;
    private final SwerveModuleOdometry frontLeftOdometry;
    private final SwerveModuleOdometry backRightOdometry;
    private final SwerveModuleOdometry backLeftOdometry;
    private final Supplier<Angle> getGyroAngle;
    private final SwerveDriveOdometry odometry;

    public SwerveChassisOdometry(
        Kinematics<SwerveState> kinematics,
        SwerveModuleOdometry frontRightOdometry,
        SwerveModuleOdometry frontLeftOdometry,
        SwerveModuleOdometry backRightOdometry,
        SwerveModuleOdometry backLeftOdometry,
        Supplier<Angle> getGyroAngle
    ) {
        this.frontRightOdometry = frontRightOdometry;
        this.frontLeftOdometry = frontLeftOdometry;
        this.backRightOdometry = backRightOdometry;
        this.backLeftOdometry = backLeftOdometry;
        this.getGyroAngle = getGyroAngle;
        this.odometry =
            new SwerveDriveOdometry(
                kinematics,
                getGyroAngle.get(),
                new PointXYZ(0, 0, 0)
            );
    }

    @Override
    public PointXYZ getRawPosition() {
        double currentTimeMs = Time.ms();
        Angle gyroAngle = getGyroAngle.get();

        SwerveModuleState frontRightState = frontRightOdometry.getState();
        SwerveModuleState frontLeftState = frontLeftOdometry.getState();
        SwerveModuleState backRightState = backRightOdometry.getState();
        SwerveModuleState backLeftState = backLeftOdometry.getState();

        SwerveState state = new SwerveState(
            frontRightState,
            frontLeftState,
            backRightState,
            backLeftState
        );

        return odometry.updateWithTime(currentTimeMs, gyroAngle, state);
    }
}
