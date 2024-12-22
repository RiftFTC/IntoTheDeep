package org.firstinspires.ftc.teamcode.util.filters;

public class ComplementaryFilter implements Filter {
    private final double alpha;
    private double filteredValue;
    private double gyroValue;
    private double accelValue;

    public ComplementaryFilter(double alpha) {
        this.alpha = alpha;
        this.filteredValue = 0;
    }

    @Override
    public double estimate(double value) {
        filteredValue = alpha * (filteredValue + gyroValue * value) + (1 - alpha) * accelValue;
        return filteredValue;
    }

    public void update(double accelValue, double gyroValue) {
        this.accelValue = accelValue;
        this.gyroValue = gyroValue;
    }
}