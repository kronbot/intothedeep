package org.firstinspires.ftc.teamcode.kronbot.utils;

import static org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants.coordinatesConvert;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.detection.GameElementDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryFactory {
    public static List<TrajectorySequence> createTrajectory(SampleMecanumDrive drive, KronBot robot, Telemetry telemetry, boolean isLeft, Runnable sleep) {
        AutonomousConstants.Coordinates pixelCoordinates;

        Pose2d startPose;

       List<TrajectorySequence> trajectories = new ArrayList<TrajectorySequence>();

        if (!isLeft) startPose = coordinatesConvert(AutonomousConstants.StartPoseRightBlue);
        else startPose = coordinatesConvert(AutonomousConstants.StartPoseLeftBlue);

        return trajectories;
    }
}
