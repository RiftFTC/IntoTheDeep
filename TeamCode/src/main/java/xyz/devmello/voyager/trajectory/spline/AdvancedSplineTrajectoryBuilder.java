/*
 * Copyright (c) 2021.
 *
 * This file is part of the "Pathfinder2" project, available here:
 * <a href="https://github.com/Wobblyyyy/Pathfinder2">GitHub</a>
 *
 * This project is licensed under the GNU GPL V3 license.
 * <a href="https://www.gnu.org/licenses/gpl-3.0.en.html">GNU GPL V3</a>
 */

package xyz.devmello.voyager.trajectory.spline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import xyz.devmello.voyager.Core;
import xyz.devmello.voyager.exceptions.InvalidSpeedException;
import xyz.devmello.voyager.exceptions.InvalidToleranceException;
import xyz.devmello.voyager.exceptions.NullAngleException;
import xyz.devmello.voyager.geometry.Angle;
import xyz.devmello.voyager.geometry.PointXYZ;
import xyz.devmello.voyager.logging.Logger;
import xyz.devmello.voyager.math.ApacheSpline;
import xyz.devmello.voyager.math.ApacheSpline.Interpolator;
import xyz.devmello.voyager.math.MonotoneCubicSpline;
import xyz.devmello.voyager.math.Spline;
import xyz.devmello.voyager.utils.StringUtils;

/**
 * A builder for the {@link AdvancedSplineTrajectory} class. Please read over
 * the documentation in {@link AdvancedSplineTrajectory} before using this
 * class!
 *
 * @author Colin Robertson
 * @since 0.6.1
 */
public class AdvancedSplineTrajectoryBuilder {
    public static final InterpolationMode DEFAULT_INTERPOLATION_MODE =
        InterpolationMode.DEFAULT;
    public static final Consumer<String> DEFAULT_LOGGER = msg -> {};

    private final List<Double> xValues = new ArrayList<>();
    private final List<Double> yValues = new ArrayList<>();
    private final List<Angle> angleTargets = new ArrayList<>();
    private final List<Double> speeds = new ArrayList<>();
    private double step = Double.MAX_VALUE;
    private double speed = Double.MAX_VALUE;
    private double tolerance = Double.MAX_VALUE;
    private Angle angleTolerance;
    private InterpolationMode interpolationMode = DEFAULT_INTERPOLATION_MODE;
    private BiFunction<Double[], Double[], Spline> customSplineGenerator = null;

    public AdvancedSplineTrajectoryBuilder() {}

    /**
     * Set the trajectory's step value.
     *
     * @param step the trajectory's step value.
     * @return {@code this}, used for method chaining.
     */
    public AdvancedSplineTrajectoryBuilder setStep(double step) {
        this.step = step;

        return this;
    }

    /**
     * Set the trajectory's speed value.
     *
     * @param speed the trajectory's speed value.
     * @return {@code this}, used for method chaining.
     */
    public AdvancedSplineTrajectoryBuilder setSpeed(double speed) {
        this.speed = speed;

        return this;
    }

    /**
     * Set the trajectory's tolerance value.
     *
     * @param tolerance the trajectory's tolerance value.
     * @return {@code this}, used for method chaining.
     */
    public AdvancedSplineTrajectoryBuilder setTolerance(double tolerance) {
        this.tolerance = tolerance;

        return this;
    }

    /**
     * Set the trajectory's angle tolerance value.
     *
     * @param angleTolerance the trajectory's angle tolerance value.
     * @return {@code this}, used for method chaining.
     */
    public AdvancedSplineTrajectoryBuilder setAngleTolerance(
        Angle angleTolerance
    ) {
        this.angleTolerance = angleTolerance;

        return this;
    }

    /**
     * Set the trajectory's interpolation mode.
     *
     * @param interpolationMode the trajectory's interpolationMode value.
     * @return {@code this}, used for method chaining.
     */
    public AdvancedSplineTrajectoryBuilder setInterpolationMode(
        InterpolationMode interpolationMode
    ) {
        this.interpolationMode = interpolationMode;

        return this;
    }

    /**
     * Set the builder's custom spline generator. If you would like to use
     * a custom implementation of a spline, you must create a function
     * that accepts two {@code Double} arrays as values and returns a new
     * spline. These two values are X and Y respectively.
     *
     * @param func the function responsible for generating a spline.
     * @return {@code this}, used for method chaining.
     */
    public AdvancedSplineTrajectoryBuilder setCustomSplineGenerator(
        BiFunction<Double[], Double[], Spline> func
    ) {
        this.customSplineGenerator = func;

        return this;
    }

    public AdvancedSplineTrajectoryBuilder add(PointXYZ target) {
        return add(target, speed);
    }

    public AdvancedSplineTrajectoryBuilder add(double x, double y) {
        return add(
            new PointXYZ(x, y, angleTargets.get(angleTargets.size() - 1)),
            speed
        );
    }

    public AdvancedSplineTrajectoryBuilder add(double x, double y, Angle z) {
        return add(new PointXYZ(x, y, z), speed);
    }

    public AdvancedSplineTrajectoryBuilder add(
        double x,
        double y,
        Angle z,
        double speed
    ) {
        return add(new PointXYZ(x, y, z), speed);
    }

    public AdvancedSplineTrajectoryBuilder add(
        double x,
        double y,
        double zDegrees,
        double speed
    ) {
        return add(new PointXYZ(x, y, zDegrees), speed);
    }

    public AdvancedSplineTrajectoryBuilder add(
        double x,
        double y,
        double speed
    ) {
        return add(
            new PointXYZ(x, y, angleTargets.get(angleTargets.size() - 1)),
            speed
        );
    }

    public AdvancedSplineTrajectoryBuilder add(PointXYZ target, double speed) {
        DEFAULT_LOGGER.accept(
            StringUtils.format(
                "--- ADDING TARGET ---\n" +
                "target x: %s\n" +
                "target y: %s\n" +
                "target z: %s\n" +
                "speed: %s",
                target.x(),
                target.y(),
                target.z(),
                speed
            )
        );

        this.speed = speed;

        xValues.add(target.x());
        yValues.add(target.y());
        angleTargets.add(target.z());
        speeds.add(speed);

        return this;
    }

    public AdvancedSplineTrajectory build() {
        boolean invalidStep = step == Double.MAX_VALUE;
        boolean invalidSpeed = speed == Double.MAX_VALUE;
        boolean invalidTolerance = tolerance == Double.MAX_VALUE;
        boolean invalidAngleTolerance = angleTolerance == null;

        DEFAULT_LOGGER.accept("--- BUILDING SPLINE ---");

        DEFAULT_LOGGER.accept(
            StringUtils.format(
                "invalid step: %s\n" +
                "invalid speed: %s\n" +
                "invalid tolerance: %s\n" +
                "invalid angle tolerance: %s",
                invalidStep,
                invalidSpeed,
                invalidTolerance,
                invalidAngleTolerance
            )
        );

        if (
            invalidStep &&
            invalidSpeed &&
            invalidTolerance &&
            invalidAngleTolerance
        ) throw new IllegalArgumentException(
            "Did not set a step, speed, tolerance, and angle tolerance " +
            "value! You need to use setStep(), setSpeed(), " +
            "setTolerance(), and setAngleTolerance() before " +
            "calling the build() method."
        );

        if (invalidStep) throw new IllegalArgumentException(
            "Did not set a step value - use setStep()."
        );

        if (invalidSpeed) throw new InvalidSpeedException(
            "Did not set a speed - use setSpeed()."
        );

        if (invalidTolerance) throw new InvalidToleranceException(
            "Did not set a tolerance - use setTolerance()."
        );

        if (invalidAngleTolerance) throw new NullAngleException(
            "Null angle tolerance while creating an " +
            "AdvancedSplineTrajectory."
        );

        int size = xValues.size();
        Double[] xBoxed = new Double[size];
        Double[] yBoxed = new Double[size];
        Double[] speedBoxed = new Double[size];
        Angle[] z = new Angle[size];

        xValues.toArray(xBoxed);
        yValues.toArray(yBoxed);
        angleTargets.toArray(z);
        speeds.toArray(speedBoxed);

        double[] x = new double[size];
        double[] y = new double[size];
        double[] speed = new double[size];
        boolean sameSpeedValue = true;

        int xDuplicates = 0;
        int yDuplicates = 0;

        for (int i = 0; i < xBoxed.length; i++) {
            x[i] = xBoxed[i];
            y[i] = yBoxed[i];
            speed[i] = speedBoxed[i];

            if (i != 0) {
                if (speed[i] != speed[i - 1]) sameSpeedValue = false;

                // ensure adjacent X values are unique
                if (x[i] == x[i - 1]) x[i] +=
                    Core.advancedSplineTrajectoryDuplicateOffset *
                    ((xDuplicates++) + 1);

                // ensure adjacent Y values are unique
                if (y[i] == y[i - 1]) y[i] +=
                    Core.advancedSplineTrajectoryDuplicateOffset *
                    ((yDuplicates++) + 1);
            }
        }

        DEFAULT_LOGGER.accept(
            StringUtils.format(
                "boxed x: %s\n" +
                "boxed y: %s\n" +
                "boxed z: %s\n" +
                "boxed speed: %s\n" +
                "unboxed x: %s\n" +
                "unboxed y: %s\n" +
                "unboxed speed: %s",
                Arrays.toString(xBoxed),
                Arrays.toString(yBoxed),
                Arrays.toString(z),
                Arrays.toString(speedBoxed),
                Arrays.toString(x),
                Arrays.toString(y),
                Arrays.toString(speed)
            )
        );

        // add support for different types of spline interpolation!
        // this is a really bad way to implement support for multiple
        // types of spline interpolation, but... oh well.
        Spline spline;
        switch (interpolationMode) {
            case DEFAULT:
                spline = new MonotoneCubicSpline(x, y);
                break;
            case CUBIC:
                spline = new ApacheSpline(Interpolator.CUBIC, x, y);
                break;
            case AKIMA:
                spline = new ApacheSpline(Interpolator.AKIMA, x, y);
                break;
            case CUSTOM:
                if (customSplineGenerator != null) spline =
                    customSplineGenerator.apply(
                        xBoxed,
                        yBoxed
                    ); else throw new NullPointerException(
                    "Tried to use custom " +
                    "spline generator without having set it first: " +
                    "use setCustomSplineGenerator() to do so. The " +
                    "function you pass in should accept two arrays " +
                    "of Double values (x and y)."
                );
                break;
            default:
                throw new RuntimeException("How did you even get here?");
        }

        AngleSpline angleSpline = new AngleSpline(x, z);

        /*
        Spline speedSpline;
        if (sameSpeedValue) speedSpline =
            new LinearSpline(
                new SlopeIntercept(0, speed[0])
            ); else speedSpline = new MonotoneCubicSpline(x, speed);
        */

        Spline speedSpline = new MonotoneCubicSpline(x, speed);

        Logger.debug(
            AdvancedSplineTrajectoryBuilder.class,
            "spline: <%s> angleSpline: <%s> speedSpline: <%s> step: <%s> " +
            "tolerance: <%s> angleTolerance: <%s>",
            spline,
            angleSpline,
            speedSpline,
            step,
            tolerance,
            angleTolerance
        );

        return new AdvancedSplineTrajectory(
            spline,
            angleSpline,
            speedSpline,
            step,
            tolerance,
            angleTolerance
        );
    }
}
