package org.firstinspires.ftc.teamcode.kronbot.utils.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class AutonomousConstants {

    public static class Coordinates {
        public double y;
        public double x;
        public double heading;

        public Coordinates(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public static Coordinates StartPoseLeftRed = new Coordinates(-35, -67 + 15/2, 270);
    public static Coordinates StartPoseRightRed = new Coordinates(12, -67 + 15/2, 270);
    public static Coordinates StartPoseLeftBlue = new Coordinates(12, 67 - 15/2, 90);
    public static Coordinates StartPoseRightBlue = new Coordinates(-35, 67 - 15/2, 90);

    public static Coordinates FirstPose = new Coordinates(-14, 0, 0);
    public static Coordinates SecondPose = new Coordinates(-14, -50.05, 0);
    public static Coordinates ThirdPose = new Coordinates(-21.49, -50.86, 0);
    public static Coordinates FourthPose = new Coordinates(-23, -4.73, 6.26);
    public static Coordinates FifthPose = new Coordinates(-22.4, -48.05, 0.04);
    public static Coordinates SixthPose = new Coordinates(-29.85, -48.77, 0.06);
    public static Coordinates SeventhPose = new Coordinates(-33.24, -5.33, 0);
    public static Coordinates EigthPose = new Coordinates(-32.28, -46.69, 0);
    public static Coordinates NinthPose = new Coordinates(-37.09, -47.38, 0);
    public static Coordinates TenthPose = new Coordinates(-38.54, -4.27, 0);
    public static Coordinates EleventhPose = new Coordinates(-38, -9, 0.07);
//    public static Coordinates TwelvethPose = new Coordinates(46, -0.8, 0.0);
//    public static Coordinates ThirdPose = new Coordinates(46, -0.8, 0.0);
    public static Coordinates Pose1 = new Coordinates(-15, 1.5, 0);
    public static Coordinates Pose2 = new Coordinates(-15, -5, 0);
    public static Coordinates Pose3 = new Coordinates(-2, 0, 0);


    public static boolean park = true;

    public static Pose2d coordinatesConvert(Coordinates coord) {
        return new Pose2d(coord.x, coord.y, Math.toRadians(coord.heading));
    }

    public static int SLEEP = 1000;
}
