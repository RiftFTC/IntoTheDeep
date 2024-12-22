package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AutoAlignIntakeCV extends OpenCvPipeline {

    public RotatedRect rect;
    public Point center;
    public Scalar avgColor;

    @Override
    public Mat processFrame(Mat input) {
        Mat gray = new Mat();
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.stackBlur(gray, gray, new Size(1.0, 1.0));

        Imgproc.erode(gray, gray, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(55, 55)));
        Imgproc.dilate(gray, gray, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(55, 55)));

        Mat edges = new Mat();

        Imgproc.Canny(gray, edges, 50, 150);

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Point imgCtr = new Point(input.cols() / 2, input.rows() / 2);
        double minDist = Double.MAX_VALUE;
        MatOfPoint2f centerRectangle = null;

        for (MatOfPoint ctr : contours) {
            // Approximate the contour to a polygon
            MatOfPoint2f ctr2f = new MatOfPoint2f(ctr.toArray());
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(ctr2f, approx, 0.02 * Imgproc.arcLength(ctr2f, true), true);

            if (approx.total() == 4) {
                // Get bounding rectangle
                rect = Imgproc.minAreaRect(approx);
                center = new Point(rect.center.x, rect.center.y);

                // Calculate distance from image center
                double distance = Math.sqrt(Math.pow(center.x - imgCtr.x, 2) + Math.pow(center.y - imgCtr.y, 2));
                if (distance < minDist) {
                    minDist = distance;
                    centerRectangle = approx;
                }
            }
        }

        Mat color = input.submat((int) center.y, (int) center.y+3, (int) center.x, (int) center.x+3);

        avgColor = Core.mean(color);

        if (centerRectangle != null) {
            MatOfPoint[] pts = new MatOfPoint[] {new MatOfPoint(centerRectangle.toArray())};
            Imgproc.drawContours(input, Arrays.asList(pts), -1, new Scalar(255, 255, 255), 1);
            Imgproc.drawMarker(input, center, new Scalar(0, 255, 0));
        }

        Imgproc.putText(input, "Angle: " + (90 - rect.angle), new Point(55, 20), 1, 1, new Scalar(255, 255, 255), 1);

        Imgproc.putText(input, "Color: " + avgColor, new Point(55, 140), 1, 1, new Scalar(255, 255, 255), 1);


        return input;
    }


    public double getOrientation() {
        return 90 - rect.angle;
    }

    public Scalar getColor() {
        return avgColor;
    }
}
