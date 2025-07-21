package org.firstinspires.ftc.teamcode.kronbot.utils.detection;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;
import com.acmerobotics.dashboard.config.Config;

@Config
public class BluePipeline extends OpenCvPipeline {


    public static double BLUE_H_MIN = 101;
    public static double BLUE_S_MIN = 101;
    public static double BLUE_V_MIN = 50;
    public static double BLUE_H_MAX = 129;
    public static double BLUE_S_MAX = 255;
    public static double BLUE_V_MAX = 255;


    public static double YELLOW_H_MIN = 20;
    public static double YELLOW_S_MIN = 100;
    public static double YELLOW_V_MIN = 100;
    public static double YELLOW_H_MAX = 40;
    public static double YELLOW_S_MAX = 255;
    public static double YELLOW_V_MAX = 255;


    public static int MIN_AREA = 1000;
    public static int MAX_AREA = 999999;

    public static int LINE_THICKNESS = 1;
    public static Scalar BLUE_RECT_COLOR = new Scalar(0, 255, 0);
    public static Scalar YELLOW_RECT_COLOR = new Scalar(0, 255, 255);
    public static boolean DRAW_CENTER = true;


    private final Mat hsvMat = new Mat();
    private final Mat blueMask = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat hierarchy = new Mat();
    private final MatOfPoint2f contour2f = new MatOfPoint2f();

    private final List<MatOfPoint> contours = new ArrayList<>();
    private final List<MatOfPoint> drawList = new ArrayList<>();

    private final Scalar blueLower = new Scalar(0, 0, 0);
    private final Scalar blueUpper = new Scalar(0, 0, 0);
    private final Scalar yellowLower = new Scalar(0, 0, 0);
    private final Scalar yellowUpper = new Scalar(0, 0, 0);

    private volatile int blueCount = 0;
    private volatile int yellowCount = 0;
    private volatile Point largestBlueCenter = null;
    private volatile Point largestYellowCenter = null;

    @Override
    public Mat processFrame(Mat input) {
        try {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);


            blueLower.set(new double[]{BLUE_H_MIN, BLUE_S_MIN, BLUE_V_MIN});
            blueUpper.set(new double[]{BLUE_H_MAX, BLUE_S_MAX, BLUE_V_MAX});
            Core.inRange(hsvMat, blueLower, blueUpper, blueMask);
            processColorMask(blueMask, input, BLUE_RECT_COLOR, true);

            yellowLower.set(new double[]{YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN});
            yellowUpper.set(new double[]{YELLOW_H_MAX, YELLOW_S_MAX, YELLOW_V_MAX});
            Core.inRange(hsvMat, yellowLower, yellowUpper, yellowMask);
            processColorMask(yellowMask, input, YELLOW_RECT_COLOR, false);

            return input;
        } catch (Exception e) {
            return input;
        }
    }

    private void processColorMask(Mat mask, Mat output, Scalar rectColor, boolean isBlue) {
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int count = 0;
        double largestArea = 0;
        Point largestCenter = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area >= MIN_AREA && area <= MAX_AREA) {
                contour.convertTo(contour2f, CvType.CV_32F);
                RotatedRect rect = Imgproc.minAreaRect(contour2f);

                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                MatOfPoint drawPoints = new MatOfPoint(rectPoints);

                drawList.clear();
                drawList.add(drawPoints);
                Imgproc.polylines(output, drawList, true, rectColor, LINE_THICKNESS);

                if (DRAW_CENTER) {
                    Imgproc.circle(output, rect.center, 8, rectColor, -1);
                }

                if (rect.size.area() > largestArea) {
                    largestArea = rect.size.area();
                    largestCenter = rect.center;
                }

                count++;
            }
        }

        if (isBlue) {
            blueCount = count;
            largestBlueCenter = largestCenter;
        } else {
            yellowCount = count;
            largestYellowCenter = largestCenter;
        }
    }

    public int getBlueCount() {
        return blueCount;
    }

    public int getYellowCount() {
        return yellowCount;
    }

    public int getTotalCount() {
        return blueCount + yellowCount;
    }

    public Point getLargestBlueCenter() {
        return largestBlueCenter;
    }

    public Point getLargestYellowCenter() {
        return largestYellowCenter;
    }
}