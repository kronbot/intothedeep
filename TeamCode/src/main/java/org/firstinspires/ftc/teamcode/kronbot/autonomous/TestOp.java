package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;

import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.FirstPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose2;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose3;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose4;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose5;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose6;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose7;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose8;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.SecondPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.ThirdPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.coordinatesConvert;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.park;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HAND_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_POWER;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Test")
public class TestOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initSimpleDriving(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d pose1=coordinatesConvert(Pose1);
        Pose2d pose2=coordinatesConvert(Pose2);
        Pose2d pose3=coordinatesConvert(Pose3);
        Pose2d pose4=coordinatesConvert(Pose4);
        Pose2d pose5=coordinatesConvert(Pose5);
        Pose2d pose6=coordinatesConvert(Pose6);
        Pose2d pose7=coordinatesConvert(Pose7);
        Pose2d pose8=coordinatesConvert(Pose8);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();


        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(pose1.getX(), pose1.getY()))
                .lineTo(new Vector2d(pose2.getX(), pose2.getY()))
                .lineTo(new Vector2d(pose3.getX(), pose3.getY()))
                .lineTo(new Vector2d(pose4.getX(), pose4.getY()))
                .lineTo(new Vector2d(pose5.getX(), pose5.getY()))
                .lineTo(new Vector2d(pose6.getX(), pose6.getY()))
                .lineTo(new Vector2d(pose7.getX(), pose7.getY()))
                .lineTo(new Vector2d(pose8.getX(), pose8.getY()))
                .build());

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
