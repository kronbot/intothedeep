package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.FifthPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.FirstPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.FourthPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose2;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose3;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.SecondPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.SeventhPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.SixthPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.ThirdPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.coordinatesConvert;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Human Zone", group = Constants.MAIN_GROUP)
public class HumanZoneOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleop(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        hardwareMap.servo.get("armRightServo").setPosition(ARM_RIGHT_MIN);
        hardwareMap.servo.get("armLeftServo").setPosition(ARM_LEFT_MIN);
        hardwareMap.servo.get("clawServo").setPosition(CLAW_CLOSE);

        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d pose1 = coordinatesConvert(FirstPose);
        Pose2d pose2 = coordinatesConvert(SecondPose);
        Pose2d pose3 = coordinatesConvert(ThirdPose);
        Pose2d pose4 = coordinatesConvert(FourthPose);
        Pose2d pose5 = coordinatesConvert(FifthPose);
        Pose2d pose6 = coordinatesConvert(SixthPose);
        Pose2d pose7 = coordinatesConvert(SeventhPose);

        drive.setPoseEstimate(startPose);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            telemetry.update();

            TrajectorySequence trajectoryToFirstPose = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(pose1.getX(), pose1.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToFirstPose);

            TrajectorySequence trajectoryToSecondPose = drive.trajectorySequenceBuilder(pose1)
                    .lineTo(new Vector2d(pose2.getX(), pose2.getY()))
                    .build();

            drive.followTrajectorySequence(trajectoryToSecondPose);
            TrajectorySequence trajectoryToThirdPose = drive.trajectorySequenceBuilder(pose2)
                    .lineTo(new Vector2d(pose3.getX(), pose3.getY()))
                    .build();

            drive.followTrajectorySequence(trajectoryToThirdPose);
            TrajectorySequence trajectoryToFourthPose = drive.trajectorySequenceBuilder(pose3)
                    .lineTo(new Vector2d(pose4.getX(), pose4.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToFourthPose);
            TrajectorySequence trajectoryToFifthPose = drive.trajectorySequenceBuilder(pose4)
                    .lineTo(new Vector2d(pose5.getX(), pose5.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToFifthPose);

            TrajectorySequence trajectorytoSixthPose = drive.trajectorySequenceBuilder(pose5)
                    .lineTo(new Vector2d(pose6.getX(), pose6.getY()))
                    .build();
            drive.followTrajectorySequence(trajectorytoSixthPose);

            TrajectorySequence trajectoryToSeventhPose = drive.trajectorySequenceBuilder(pose6)
                    .lineTo(new Vector2d(pose7.getX(), pose7.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToSeventhPose);
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
