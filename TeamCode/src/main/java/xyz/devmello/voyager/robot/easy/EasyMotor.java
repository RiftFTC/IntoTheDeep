/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.robot.easy;

import java.util.function.Consumer;
import java.util.function.Supplier;
import xyz.devmello.voyager.robot.components.AbstractMotor;
import xyz.devmello.voyager.robot.components.Motor;

/**
 * Static methods to create {@link Motor} instances.
 *
 * @author Colin Robertson
 * @since 0.1.0
 */
public class EasyMotor {

    private EasyMotor() {}

    public static Motor buildMotor(
        Consumer<Double> setPower,
        Supplier<Double> getPower,
        boolean invertSetPower
    ) {
        return buildMotor(setPower, getPower, invertSetPower, invertSetPower);
    }

    public static Motor buildMotor(
        Consumer<Double> setPower,
        Supplier<Double> getPower,
        boolean invertSetPower,
        boolean invertGetPower
    ) {
        return new AbstractMotor(
            setPower,
            getPower,
            invertSetPower,
            invertGetPower
        );
    }

    public static Motor buildMotor(
        Consumer<Double> setPower,
        Supplier<Double> getPower
    ) {
        return new AbstractMotor(setPower, getPower);
    }

    public static Motor buildMotor(Consumer<Double> setPower) {
        return new Motor() {
            private double power;

            @Override
            public double getPower() {
                return power;
            }

            @Override
            public void setPower(double power) {
                setPower.accept(power);
                this.power = power;
            }
        };
    }
}
