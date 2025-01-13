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

import java.util.function.Function;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.geometry.Translation;
import xyz.devmello.voyager.robot.Drive;
import xyz.devmello.voyager.robot.Odometry;
import xyz.devmello.voyager.robot.Robot;

/**
 * An implementation of {@link Drive} and {@link Odometry} that allows
 * a translation to be applied to modify the simulated robot's position.
 *
 * @author Colin Robertson
 * @since 1.4.2
 */
public class SimulatedRobot extends Robot implements Drive, Odometry {
    private Translation translation = Translation.ZERO;
    private PointXYZ offset = PointXYZ.zero();
    private PointXYZ position = PointXYZ.zero();
    private Function<Translation, Translation> dm = t -> t;

    @Override
    public void setTranslation(Translation translation) {
        translation = dm.apply(translation);
        this.translation = translation;
        position = position.applyTranslation(translation);
    }

    @Override
    public Translation getTranslation() {
        return translation;
    }

    @Override
    public PointXYZ getRawPosition() {
        return position;
    }

    public void setPosition(PointXYZ position) {
        this.position = position;
    }

    public void setPosition(double x, double y) {
        setPosition(new PointXYZ(x, y, position.z()));
    }

    public void setPosition(double x, double y, Angle z) {
        setPosition(new PointXYZ(x, y, z));
    }

    public void setPosition(double x, double y, double z) {
        setPosition(x, y, Angle.fromDeg(z));
    }

    @Override
    public Drive drive() {
        return this;
    }

    @Override
    public Odometry odometry() {
        return this;
    }

    @Override
    public void setOffset(PointXYZ offset) {
        this.offset = offset;
    }

    @Override
    public PointXYZ getOffset() {
        return offset;
    }

    @Override
    public void setDriveModifier(Function<Translation, Translation> dm) {
        this.dm = dm;
    }

    @Override
    public Function<Translation, Translation> getDriveModifier() {
        return dm;
    }
}
