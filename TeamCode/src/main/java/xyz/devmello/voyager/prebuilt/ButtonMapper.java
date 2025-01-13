/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.prebuilt;

import xyz.devmello.voyager.Voyager;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.trajectory.LinearTrajectory;
import xyz.devmello.voyager.trajectory.Trajectory;
import xyz.devmello.voyager.utils.Gamepad;

/**
 * @deprecated use the listener system instead!
 */
@Deprecated
public class ButtonMapper {
    private final Voyager voyager;
    private final Gamepad gamepad;

    public ButtonMapper(Voyager voyager, Gamepad gamepad) {
        this.voyager = voyager;
        this.gamepad = gamepad;
    }

    public Trajectory getTrajectory() {
        if (!gamepad.areAnyButtonsPressed()) throw new RuntimeException(
            "Cannot get a trajectory without any buttons being pressed!"
        );

        PointXYZ point = null;
        if (gamepad.a()) point =
            gamepad.getMappedPoint(Gamepad.InputButton.BUTTON_A); else if (
            gamepad.b()
        ) point =
            gamepad.getMappedPoint(Gamepad.InputButton.BUTTON_B); else if (
            gamepad.x()
        ) point =
            gamepad.getMappedPoint(Gamepad.InputButton.BUTTON_X); else if (
            gamepad.y()
        ) point = gamepad.getMappedPoint(Gamepad.InputButton.BUTTON_Y);
        if (point != null) {
            double speed = voyager.getSpeed();
            double tolerance = voyager.getTolerance();
            Angle angleTolerance = voyager.getAngleTolerance();

            return new LinearTrajectory(
                point,
                speed,
                tolerance,
                angleTolerance
            );
        }

        return null;
    }

    public void update() {
        if (gamepad.areAnyButtonsPressed()) {
            Trajectory trajectory = getTrajectory();

            voyager.followTrajectory(trajectory);
        }

        if (gamepad.start()) {
            voyager.clear();
        }
    }
}
