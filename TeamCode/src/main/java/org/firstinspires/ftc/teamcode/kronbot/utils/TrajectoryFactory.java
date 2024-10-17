package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.coordinatesConvert;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_REVERSE_CONSTANT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.REST_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDES_SPEED;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryFactory {
    public static List<TrajectorySequence> createTrajectory(SampleMecanumDrive drive, GameElementDetection.Position position, KronBot robot, Telemetry telemetry, Runnable sleep, boolean isBlue, boolean isClose) {
        AutonomousConstants.Coordinates pixelCoordinates;

        Pose2d startPose;

       List<TrajectorySequence> trajectories = new ArrayList<TrajectorySequence>();

        int multiplier = isBlue ? -1 : 1;

        if (isBlue) {
            if (!isClose) startPose = coordinatesConvert(AutonomousConstants.StartPoseRightBlue);
            else startPose = coordinatesConvert(AutonomousConstants.StartPoseLeftBlue);
        } else {
            if (isClose) startPose = coordinatesConvert(AutonomousConstants.StartPoseRightRed);
            else startPose = coordinatesConvert(AutonomousConstants.StartPoseLeftRed);
        }

        return trajectories;
    }
}
