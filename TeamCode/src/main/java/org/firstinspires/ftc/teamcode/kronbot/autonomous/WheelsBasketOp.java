package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose2;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose3;
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

@Autonomous(name = "Wheels Basket", group = Constants.MAIN_GROUP)
public class WheelsBasketOp extends LinearOpMode {
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

        drive.setPoseEstimate(startPose);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            telemetry.update();

            TrajectorySequence trajectoryToPose1 = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(pose1.getX(), pose1.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToPose1);

            TrajectorySequence trajectoryToPose2 = drive.trajectorySequenceBuilder(pose1)
                    .lineTo(new Vector2d(pose2.getX(), pose2.getY()))
                    .turn(Math.toRadians(25))
                    .build();

            drive.followTrajectorySequence(trajectoryToPose2);

            robot.liftLeft.setTargetPosition(2500);
            robot.liftRight.setTargetPosition(2500);

            robot.liftLeft.setPower(1.0);
            robot.liftRight.setPower(1.0);

            sleep(500);
            robot.armRight.setPosition(Constants.ARM_RIGHT_MAX);
            robot.armLeft.setPosition(Constants.ARM_LEFT_MAX);

            sleep(500);
            hardwareMap.servo.get("clawServo").setPosition(Constants.CLAW_OPEN); //drop sample in basket

            //retract lift and arm
            hardwareMap.servo.get("clawServo").setPosition(CLAW_CLOSE);
            hardwareMap.servo.get("armLeftServo").setPosition(ARM_LEFT_MIN);
            hardwareMap.servo.get("armRightServo").setPosition(ARM_RIGHT_MIN);
            sleep(100);
            while (robot.liftLeft.isBusy() && robot.liftRight.isBusy() && opModeIsActive()) {
                telemetry.addData("Lift Left Pos", robot.liftLeft.getCurrentPosition());
                telemetry.addData("Lift Right Pos", robot.liftRight.getCurrentPosition());
                telemetry.update();
            }
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);

            //go to sample 2
            TrajectorySequence trajectoryToPose3 = drive.trajectorySequenceBuilder(pose2)
                    .lineTo(new Vector2d(pose3.getX(), pose3.getY()))
                    .turn(Math.toRadians(-75))
                    .build();

            drive.followTrajectorySequence(trajectoryToPose3);
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
