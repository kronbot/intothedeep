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

   //constante pt high rung + 3 monstre in observation
    public static Coordinates PoseA = new Coordinates(-24, 2.9, 0.036);
    public static Coordinates PoseB = new Coordinates(-20, 2.9, 0.003);
    public static Coordinates FirstPose = new Coordinates(-20, 27.5, 0.01);
    public static Coordinates SecondPose = new Coordinates(-47, 27.5, 0.01);
    public static Coordinates ThirdPose = new Coordinates(-47, 37.0, 0.01);
    public static Coordinates FourthPose = new Coordinates(-1.5, 37.5, 0.01);
    public static Coordinates FifthPose = new Coordinates(-47, 37.5, 0.01);
    public static Coordinates SixthPose = new Coordinates(-47, 44.5, 0.01);
    public static Coordinates SeventhPose = new Coordinates(-1.5, 44.5, 0.01);
    public static Coordinates EigthPose = new Coordinates(-47, 44.5, 0.01);
    public static Coordinates NinthPose = new Coordinates(-47, 51, 0.01);
    public static Coordinates TenthPose = new Coordinates(-3.7, 51, 0.01);
    public static Coordinates EleventhPose = new Coordinates(-9.5, 51, 0.01);

    //high rung+parcare sub basket
    public static Coordinates PoseNr1 = new Coordinates(0, 0, 0.01);
    public static Coordinates PoseNr2 = new Coordinates(0, -37, 0.01);

    // constante pt high rung + 3 monstre sub basket

    public static Coordinates FirstPose2 = new Coordinates(-20, -27.5, 0.01);
    public static Coordinates SecondPose2 = new Coordinates(-47, -27.5, 0.01);
    public static Coordinates ThirdPose2 = new Coordinates(-47, -37.0, 0.01);
    public static Coordinates FourthPose2 = new Coordinates(-1.5, -37.5, 0.01);
    public static Coordinates FifthPose2 = new Coordinates(-47, -37.5, 0.01);
    public static Coordinates SixthPose2 = new Coordinates(-47, -44.5, 0.01);
    public static Coordinates SeventhPose2 = new Coordinates(-1.5, -44.5, 0.01);
    public static Coordinates EigthPose2 = new Coordinates(-47, -44.5, 0.01);
    public static Coordinates NinthPose2 = new Coordinates(-47, -51, 0.01);
    public static Coordinates TenthPose2 = new Coordinates(-3.7, -51, 0.01);
    public static Coordinates EleventhPose2 = new Coordinates(-9.5, -51, 0.01);





//    public static Coordinates TwelvethPose = new Coordinates(11.4, 70, 0.07);
    public static int LIFT_AUTO=650;

    //SpecimensOp
    public static Coordinates Pose1 = new Coordinates(-15.68, 0, 0);
    public static Coordinates Pose2 = new Coordinates(-14, -47.32, 0);
    public static Coordinates Pose3 = new Coordinates(-22.66, -47.62, 0);
    public static Coordinates Pose4 = new Coordinates(-25, -5.8, 0);
    public static Coordinates Pose5 = new Coordinates(-22.66, -47.72, 0);
    public static Coordinates Pose6 = new Coordinates(-31.74, -48.72, 0);
    public static Coordinates Pose7 =  new Coordinates(-33.24, -5.6, 0);
    public static Coordinates Pose8 = new Coordinates(-32.28, -48.43, 0);
    public static Coordinates Pose9 = new Coordinates(-35.82, -48.6, 0);
    public static Coordinates Pose10 = new Coordinates(-37.77, -4.5, 0);
    public static Coordinates Pose11 = new Coordinates(-37.25, -9.48, 0.07);




    public static boolean park = true;

    public static Pose2d coordinatesConvert(Coordinates coord) {
        return new Pose2d(coord.x, coord.y, Math.toRadians(coord.heading));
    }

    public static int SLEEP = 1000;
}
