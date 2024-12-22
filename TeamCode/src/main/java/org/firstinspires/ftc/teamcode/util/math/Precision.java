package org.firstinspires.ftc.teamcode.util.math;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.math.BigDecimal;

public class Precision {
    public static double clip(double value, double min, double max) {return Math.min(Math.max(value, min), max);}
    public static double clip(double value, double range){ return clip(value, -range, range); }

    public static double round(double x, int scale) {
        return round(x, scale, BigDecimal.ROUND_HALF_UP);
    }
    public static double round(double x, int scale, int roundingMethod) {
        try {
            final double rounded = (new BigDecimal(Double.toString(x))
                    .setScale(scale, roundingMethod))
                    .doubleValue();
            return rounded == 0d ? 0d * x : rounded;
        } catch (NumberFormatException ex) {
            if (Double.isInfinite(x)) {
                return x;
            } else {
                return Double.NaN;
            }
        }
    }

    public static double calculateWeightedValue(double limitLeft, double limitRight, double weight) {
        if (weight < 0 || weight > 1) return -1;
        return limitLeft + (limitRight - limitLeft) * (1 - weight);
    }

    public static double limit(double percent, double lowerLimit, double upperLimit) {
        return Precision.clip(percent, 0, 1) * (upperLimit - lowerLimit) + lowerLimit;

    }

    public static Pose2d toPose2d(Pose2D pose2D) {
        return new Pose2d(pose2D.getX(DistanceUnit.INCH), pose2D.getY(DistanceUnit.INCH), pose2D.getHeading(AngleUnit.RADIANS));
    }

    public static Pose2D fromPose2d(Pose2d pose2d) {
        return new Pose2D(DistanceUnit.INCH, pose2d.position.x, pose2d.position.y, AngleUnit.RADIANS, pose2d.heading.toDouble());
    }

}
