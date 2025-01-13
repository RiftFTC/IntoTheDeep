/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.math;

import xyz.devmello.voyager.geometry.LinearEquation;
import xyz.devmello.voyager.geometry.PointXY;
import xyz.devmello.voyager.utils.StringUtils;

/**
 * A spline based on a linear equation.
 *
 * @author Colin Robertson
 * @since 0.6.1
 */
public class LinearSpline implements Spline {
    private final LinearEquation equation;

    public LinearSpline(LinearEquation equation) {
        this.equation = equation;
    }

    @Override
    public double interpolateY(double x) {
        return equation.getY(x);
    }

    @Override
    public PointXY interpolate(double x) {
        return new PointXY(x, this.equation.getY(x));
    }

    @Override
    public PointXY getStartPoint() {
        return interpolate(0);
    }

    @Override
    public PointXY getEndPoint() {
        return interpolate(0);
    }

    @Override
    public String toString() {
        return StringUtils.format(
            "LinearSpline (slope: <%s>)",
            equation.getSlope()
        );
    }
}
