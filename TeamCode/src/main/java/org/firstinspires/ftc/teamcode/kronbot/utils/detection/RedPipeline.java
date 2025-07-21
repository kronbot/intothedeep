package org.firstinspires.ftc.teamcode.kronbot.utils.detection;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import org.openftc.easyopencv.OpenCvPipeline;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RedPipeline extends OpenCvPipeline {

    public static double RED_H_MIN_1 = 0;
    public static double RED_H_MAX_1 = 10;
    public static double RED_H_MIN_2 = 160;
    public static double RED_H_MAX_2 = 180;
    public static double RED_S_MIN = 100;
    public static double RED_S_MAX = 255;
    public static double RED_V_MIN = 50;
    public static double RED_V_MAX = 255;

    public static double YELLOW_H_MIN = 20;
    public static double YELLOW_S_MIN = 100;
    public static double YELLOW_V_MIN = 100;
    public static double YELLOW_H_MAX = 40;
    public static double YELLOW_S_MAX = 255;
    public static double YELLOW_V_MAX = 255;

    public static int MIN_AREA = 1000;
    public static int MAX_AREA = 999999;

    public static int LINE_THICKNESS = 1;
    public static Scalar RED_RECT_COLOR = new Scalar(255, 0, 0);
    public static Scalar YELLOW_RECT_COLOR = new Scalar(0, 255, 255);
    public static boolean DRAW_CENTER = true;

    private final Mat hsvMat = new Mat();
    private final Mat redMask1 = new Mat();
    private final Mat redMask2 = new Mat();
    private final Mat redMask = new Mat();
    private final Mat yellowMask = new Mat();
    private final Mat hierarchy = new Mat();
    private final MatOfPoint2f contour2f = new MatOfPoint2f();

    private final List<MatOfPoint> contours = new ArrayList<>();
    private final List<MatOfPoint> drawList = new ArrayList<>();

    private final Scalar redLower1 = new Scalar(0, 0, 0);
    private final Scalar redUpper1 = new Scalar(0, 0, 0);
    private final Scalar redLower2 = new Scalar(0, 0, 0);
    private final Scalar redUpper2 = new Scalar(0, 0, 0);
    private final Scalar yellowLower = new Scalar(0, 0, 0);
    private final Scalar yellowUpper = new Scalar(0, 0, 0);

    private volatile int redCount = 0;
    private volatile int yellowCount = 0;
    private volatile Point largestRedCenter = null;
    private volatile Point largestYellowCenter = null;

    @Override
    public Mat processFrame(Mat input) {
        try {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            redLower1.set(new double[]{RED_H_MIN_1, RED_S_MIN, RED_V_MIN});
            redUpper1.set(new double[]{RED_H_MAX_1, RED_S_MAX, RED_V_MAX});
            Core.inRange(hsvMat, redLower1, redUpper1, redMask1);

            redLower2.set(new double[]{RED_H_MIN_2, RED_S_MIN, RED_V_MIN});
            redUpper2.set(new double[]{RED_H_MAX_2, RED_S_MAX, RED_V_MAX});
            Core.inRange(hsvMat, redLower2, redUpper2, redMask2);

            Core.bitwise_or(redMask1, redMask2, redMask);
            processColorMask(redMask, input, RED_RECT_COLOR, true);

            yellowLower.set(new double[]{YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN});
            yellowUpper.set(new double[]{YELLOW_H_MAX, YELLOW_S_MAX, YELLOW_V_MAX});
            Core.inRange(hsvMat, yellowLower, yellowUpper, yellowMask);
            processColorMask(yellowMask, input, YELLOW_RECT_COLOR, false);

            return input;
        } catch (Exception e) {
            return input;
        }
    }

    private void processColorMask(Mat mask, Mat output, Scalar rectColor, boolean isRed) {
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

        if (isRed) {
            redCount = count;
            largestRedCenter = largestCenter;
        } else {
            yellowCount = count;
            largestYellowCenter = largestCenter;
        }
    }

    public int getRedCount() {
        return redCount;
    }

    public int getYellowCount() {
        return yellowCount;
    }

    public int getTotalCount() {
        return redCount + yellowCount;
    }

    public Point getLargestRedCenter() {
        return largestRedCenter;
    }

    public Point getLargestYellowCenter() {
        return largestYellowCenter;
    }
}