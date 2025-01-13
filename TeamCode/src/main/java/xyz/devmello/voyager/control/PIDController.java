/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.control;

import com.stormbots.MiniPID;

/**
 * A proportional, integral, derivative controller. This is like the
 * {@link ProportionalController}'s cool older cousin. Not only does it
 * factor in proportion, it even brings in the integral and derivative
 * components as well! And you thought you'd never use calculus...
 *
 * @author Colin Robertson
 * @since 0.15.0
 */
public class PIDController extends AbstractController {
    private final MiniPID pid;

    public PIDController(double p, double i, double d) {
        this(p, i, d, 0.02);
    }

    public PIDController(double p, double i, double d, double f) {
        pid = new MiniPID(p, i, d, f);
    }

    @Override
    public double calculate(double value) {
        return clip(pid.getOutput(value, getTarget()));
    }

    public PIDController setP(double p) {
        pid.setP(p);

        return this;
    }

    public PIDController setI(double i) {
        pid.setI(i);

        return this;
    }

    public PIDController setD(double d) {
        pid.setDerivative(d);

        return this;
    }

    public PIDController setF(double f) {
        pid.setFeedForward(f);

        return this;
    }
}
