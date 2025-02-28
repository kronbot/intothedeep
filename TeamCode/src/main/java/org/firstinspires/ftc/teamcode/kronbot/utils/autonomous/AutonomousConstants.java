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

    public static Coordinates FirstPose = new Coordinates(-0.4, 50, 0);
    public static Coordinates SecondPose = new Coordinates(-8.4, 50, 0);
    public static Coordinates ThirdPose = new Coordinates(-7.6, 0, 6.2);
    public static Coordinates FourthPose = new Coordinates(-9.6, 50, 0);
    public static Coordinates FifthPose = new Coordinates(-18, 50, 0.01);
    public static Coordinates SixthPose = new Coordinates(-17.5, 3, 6.2);
    public static Coordinates SeventhPose = new Coordinates(-17.4, 50, 0.02);
    public static Coordinates EigthPose = new Coordinates(-23.1, 50, 0.02);
    public static Coordinates NinthPose = new Coordinates(-21.9, 8.91, 0.05);
    public static Coordinates TenthPose = new Coordinates(-21.6, 50, 0.04);
    public static Coordinates EleventhPose = new Coordinates(11.4, 70, 0.07);
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
