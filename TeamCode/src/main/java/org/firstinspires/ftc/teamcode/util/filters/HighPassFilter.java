package org.firstinspires.ftc.teamcode.util.filters;

public class HighPassFilter implements Filter {
    private double alpha;
    private double previousValue = 0;
    private double previousFilteredValue = 0;

    public HighPassFilter(double alpha) {
        this.alpha = alpha;
    }

    @Override
    public double estimate(double newValue) {
        double filteredValue = alpha * (previousFilteredValue + newValue - previousValue);
        previousValue = newValue;
        previousFilteredValue = filteredValue;
        return filteredValue;
    }
}