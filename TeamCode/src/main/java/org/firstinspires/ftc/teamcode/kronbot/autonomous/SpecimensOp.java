package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose10;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose11;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose2;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose3;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose4;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose5;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose6;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose7;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose8;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose9;
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

@Autonomous(name = "Specimens", group = Constants.MAIN_GROUP)
public class SpecimensOp extends LinearOpMode {
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

        Pose2d pose1 = coordinatesConvert(Pose1);
        Pose2d pose2 = coordinatesConvert(Pose2);
        Pose2d pose3 = coordinatesConvert(Pose3);
        Pose2d pose4 = coordinatesConvert(Pose4);
        Pose2d pose5 = coordinatesConvert(Pose5);
        Pose2d pose6 = coordinatesConvert(Pose6);
        Pose2d pose7 = coordinatesConvert(Pose7);
        Pose2d pose8 = coordinatesConvert(Pose8);
        Pose2d pose9 = coordinatesConvert(Pose9);
        Pose2d pose10 = coordinatesConvert(Pose10);
//        Pose2d pose11 = coordinatesConvert(Pose11);

        drive.setPoseEstimate(startPose);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_CLOSED);
            robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_CLOSED);
            robot.intakeServoRight.setPosition(INTAKE_RIGHT_MAX);
            robot.intakeServoLeft.setPosition(INTAKE_LEFT_MAX);
            hardwareMap.servo.get("armRightServo").setPosition(ARM_RIGHT_MIN);
            hardwareMap.servo.get("armLeftServo").setPosition(ARM_LEFT_MIN);
            robot.claw.setPosition(CLAW_CLOSE);
        }

        waitForStart();



        if (opModeIsActive()) {
            telemetry.update();

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

            TrajectorySequence trajectoryToEightPose = drive.trajectorySequenceBuilder(pose7)
                    .lineTo(new Vector2d(pose8.getX(), pose8.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToEightPose);

            TrajectorySequence trajectoryToNinthPose = drive.trajectorySequenceBuilder(pose8)
                    .lineTo(new Vector2d(pose9.getX(), pose9.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToNinthPose);

            TrajectorySequence trajectoryToTenthPose = drive.trajectorySequenceBuilder(pose9)
                    .lineTo(new Vector2d(pose10.getX(), pose10.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToTenthPose);
//
//            TrajectorySequence trajectoryToEleventhPose = drive.trajectorySequenceBuilder(pose10)
//                    .lineTo(new Vector2d(pose11.getX(), pose11.getY()))
//                    .build();
//            drive.followTrajectorySequence(trajectoryToEleventhPose);------

//            TrajectorySequence trajectoryToTwelvethPose = drive.trajectorySequenceBuilder(pose11)
//                    .lineTo(new Vector2d(pose12.getX(), pose12.getY()))
//                    .build();
//            drive.followTrajectorySequence(trajectoryToTwelvethPose);
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}