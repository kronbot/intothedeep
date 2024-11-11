package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;

import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.FirstPose;
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

@Autonomous(name = "Demo")
public class DemoOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAutonomy(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d firstPose = coordinatesConvert(FirstPose);
        Pose2d secondPose = coordinatesConvert(SecondPose);
        Pose2d thirdPose = coordinatesConvert(ThirdPose);

        robot.claw.setPosition(CLAW_CLOSE);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        robot.hand.setPosition(HAND_MAX);
        robot.armLeft.setPosition(ARM_LEFT_MIN);
        robot.armRight.setPosition(ARM_RIGHT_MIN);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(firstPose.getX(), firstPose.getY())).build());

        robot.lift.setTargetPosition(LIFT_MAX_POSITION);
        robot.lift.setPower(LIFT_POWER);

        sleep(3000);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(firstPose)
                .lineTo(new Vector2d(secondPose.getX(), secondPose.getY())).build());

        robot.claw.setPosition(CLAW_OPEN);

        sleep(500);

        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(secondPose)
                .lineTo(new Vector2d(secondPose.getX() - 3, secondPose.getY())).build());

        robot.lift.setTargetPosition(LIFT_INIT_POSITION);
        robot.lift.runToPosition();

        sleep(3000);

        if (park)
            drive.followTrajectorySequence(drive.trajectorySequenceBuilder(secondPose)
                .lineTo(new Vector2d(thirdPose.getX(), thirdPose.getY())).build());

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
