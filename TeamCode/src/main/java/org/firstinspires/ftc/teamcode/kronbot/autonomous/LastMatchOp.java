package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.EleventhPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.FifthPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.FirstPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.FourthPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.LIFT_AUTO;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.NinthPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.PoseA;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.PoseB;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.PoseNr1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.PoseNr2;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.SecondPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.SeventhPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.SixthPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.TenthPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.ThirdPose;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.EigthPose;
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

@Autonomous(name = "Last Match", group = Constants.MAIN_GROUP)
public class LastMatchOp extends LinearOpMode {
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

        Pose2d poseA = coordinatesConvert(PoseA);
        Pose2d poseB = coordinatesConvert(PoseB);
        Pose2d pose1 = coordinatesConvert(PoseNr1);
        Pose2d pose2 = coordinatesConvert(PoseNr2);


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

             sleep(4000);
            TrajectorySequence trajectoryToPoseA = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(poseA.getX(), poseA.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToPoseA);


            robot.liftLeft.setTargetPosition(LIFT_AUTO);
            robot.liftRight.setTargetPosition(LIFT_AUTO);
            while(robot.liftLeft.getCurrentPosition()<LIFT_AUTO && robot.liftRight.getCurrentPosition()<LIFT_AUTO) {
                robot.liftLeft.setPower(1.0);
                robot.liftRight.setPower(1.0);
            }
            telemetry.addData("plang", "jbn");
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);

            robot.armRight.setPosition(Constants.ARM_RIGHT_MAX);
            robot.armLeft.setPosition(Constants.ARM_LEFT_MAX);

            while (robot.liftLeft.isBusy() && robot.liftRight.isBusy() && opModeIsActive()) {
                telemetry.addData("Lift Left Pos", robot.liftLeft.getCurrentPosition());
                telemetry.addData("Lift Right Pos", robot.liftRight.getCurrentPosition());
                telemetry.update();
            }

            sleep(1000);
            TrajectorySequence trajectoryToPoseB = drive.trajectorySequenceBuilder(poseA)
                    .lineTo(new Vector2d(poseB.getX(), poseB.getY()))
                    .build();

            drive.followTrajectorySequence(trajectoryToPoseB); //move forward

            sleep(500);
            hardwareMap.servo.get("clawServo").setPosition(Constants.CLAW_OPEN);

            sleep(500);
            hardwareMap.servo.get("armLeftServo").setPosition(ARM_LEFT_MIN);
            hardwareMap.servo.get("armRightServo").setPosition(ARM_RIGHT_MIN);

            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);
            robot.liftLeft.setPower(-1);
            robot.liftRight.setPower(-1);

            while (robot.liftLeft.isBusy() && robot.liftRight.isBusy() && opModeIsActive()) {
                telemetry.addData("Lift Left Pos", robot.liftLeft.getCurrentPosition());
                telemetry.addData("Lift Right Pos", robot.liftRight.getCurrentPosition());
                telemetry.update();
            }
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);

            //begin to drag samples under the basket
            TrajectorySequence trajectoryToFirstPose = drive.trajectorySequenceBuilder(poseB)
                    .lineTo(new Vector2d(pose1.getX(), pose1.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToFirstPose);

            TrajectorySequence trajectoryToSecondPose = drive.trajectorySequenceBuilder(pose1)
                    .lineTo(new Vector2d(pose2.getX(), pose2.getY()))
                    .build();
            drive.followTrajectorySequence(trajectoryToSecondPose);
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}