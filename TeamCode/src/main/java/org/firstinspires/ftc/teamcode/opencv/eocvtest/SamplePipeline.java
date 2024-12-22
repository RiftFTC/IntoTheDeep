package org.firstinspires.ftc.teamcode.opencv.eocvtest;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.*;

public class SamplePipeline extends OpenCvPipeline {

    public enum TEAM {
        RED,
        BLUE
    }

    private final double gain = 0.8;
    private double prevValue;
    public final MovingAverageFilter angleFilter = new MovingAverageFilter(15);

    Telemetry telemetry;

    public SamplePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static double sizeThreshold = 3000;  // This value can be tuned based on your needs

    public final TEAM team = TEAM.RED;
    private boolean drawOnScreen = true;

    Mat ycrcbMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();
    Mat blueThresholdMat = new Mat();
    Mat redThresholdMat = new Mat();
    Mat yellowThresholdMat = new Mat();

    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();

    static final int YELLOW_MASK_THRESHOLD = 80;
    static final int BLUE_MASK_THRESHOLD = 150;
    static final int RED_MASK_THRESHOLD = 180;

    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar YELLOW = new Scalar(255, 255, 0);

    static class AnalyzedStone {
        double angle;
        String color;
        Point center;  // Added center to track proximity to the screen center
    }

    ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        internalStoneList.clear();
        findContours(input);
        clientStoneList = new ArrayList<>(internalStoneList);
        telemetry.addData("Detected stones", clientStoneList.size());
        telemetry.addData("Angle", getAngle(input.size()));
        telemetry.addData("Height Width",input.size().height + " " + input.size().width);
        telemetry.update();
        return input;
    }

    void findContours(Mat input) {
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(ycrcbMat, cbMat, 2);
        Core.extractChannel(ycrcbMat, crMat, 1);
        contoursOnPlainImageMat = Mat.zeros(input.size(), input.type());
        Imgproc.threshold(cbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
        morphMask(yellowThresholdMat, morphedYellowThreshold);
        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        for (MatOfPoint contour : yellowContoursList) {
            analyzeContour(contour, input, "Yellow");
        }
        if (team == TEAM.BLUE) {
            Imgproc.threshold(cbMat, blueThresholdMat, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
            morphMask(blueThresholdMat, morphedBlueThreshold);
            ArrayList<MatOfPoint> blueContoursList = new ArrayList<>();
            Imgproc.findContours(morphedBlueThreshold, blueContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            for (MatOfPoint contour : blueContoursList) {
                analyzeContour(contour, input, "Blue");
            }
        } else {
            Imgproc.threshold(crMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
            morphMask(redThresholdMat, morphedRedThreshold);
            ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
            Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            for (MatOfPoint contour : redContoursList) {
                analyzeContour(contour, input, "Red");
            }
        }
    }

    void morphMask(Mat input, Mat output) {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);
        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input, String color) {
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        // Calculate the size of the rectangle (width * height)
        double rectangleSize = rotatedRectFitToContour.size.width * rotatedRectFitToContour.size.height;

        // Set a size threshold to filter out small rectangles


        if (rectangleSize > sizeThreshold) {
            // Draw the rectangle and text on the screen
            if (drawOnScreen) {
                drawRotatedRect(rotatedRectFitToContour, input, color);
                drawRotatedRect(rotatedRectFitToContour, contoursOnPlainImageMat, color);
            }

            double rotRectAngle = rotatedRectFitToContour.angle;
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
                rotRectAngle += 90;
            }

            double angle = -(rotRectAngle - 180);
            if (drawOnScreen) {
                drawTagText(rotatedRectFitToContour, (int) Math.round(angle) + " deg", input, color);
            }

            AnalyzedStone analyzedStone = new AnalyzedStone();
            analyzedStone.angle = Math.round(angle);
            analyzedStone.color = color;
            analyzedStone.center = rotatedRectFitToContour.center; // Store center of rectangle
            internalStoneList.add(analyzedStone);

            // Add telemetry to show the rectangle size
            telemetry.addData("Rectangle Size (" + color + ")", rectangleSize);
        } else {
            // You can optionally show a telemetry message for skipped small rectangles
            telemetry.addData("Skipped small rectangle (" + color + ")", rectangleSize);
        }
    }

    static void drawTagText(RotatedRect rect, String text, Mat mat, String color) {
        Scalar colorScalar = getColorScalar(color);

        Imgproc.putText(
                mat,
                text,
                new Point(
                        rect.center.x - 50,
                        rect.center.y + 25),
                Imgproc.FONT_HERSHEY_PLAIN,
                1,
                colorScalar,
                1);
    }

    static void drawRotatedRect(RotatedRect rect, Mat drawOn, String color) {
        Point[] points = new Point[4];
        rect.points(points);

        Scalar colorScalar = getColorScalar(color);

        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], colorScalar, 2);
        }
    }

    static Scalar getColorScalar(String color) {
        switch (color) {
            case "Blue":
                return BLUE;
            case "Yellow":
                return YELLOW;
            default:
                return RED;
        }
    }

    public double getAngle(Size frameSize) {
        if (clientStoneList.isEmpty()) {
            return -1;
        }

        Point frameCenter = new Point(frameSize.width / 2, frameSize.height / 2);
        AnalyzedStone closestStone = null;
        double closestDistance = Double.MAX_VALUE;

        for (AnalyzedStone stone : clientStoneList) {
            double distanceToCenter = Math.hypot(stone.center.x - frameCenter.x, stone.center.y - frameCenter.y);
            if (distanceToCenter < closestDistance) {
                closestDistance = distanceToCenter;
                closestStone = stone;
            }
        }

        if (closestStone != null) {
            return angleFilter.filter(closestStone.angle);
        } else {
            return -1;
        }
    }

    public void setDrawOnScreen(boolean enabled) {
        this.drawOnScreen = enabled;
    }

    public double estimate(double value) {
        prevValue = gain * value + (1 - gain) * prevValue;
        return prevValue;
    }

    static class MovingAverageFilter {
        private final int windowSize;
        private final Queue<Double> window = new LinkedList<>();
        private double sum = 0.0;

        public MovingAverageFilter(int windowSize) {
            this.windowSize = windowSize;
        }

        public double filter(double newValue) {
            sum += newValue;
            window.add(newValue);

            // Remove the oldest value if the window is full
            if (window.size() > windowSize) {
                sum -= window.poll();
            }

            // Return the average of the values in the window
            return sum / window.size();
        }
    }
}
