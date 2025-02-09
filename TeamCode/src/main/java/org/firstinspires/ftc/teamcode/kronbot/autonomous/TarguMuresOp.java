package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose2;
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

@Autonomous(name = "Targu Mures", group = Constants.MAIN_GROUP)
public class TarguMuresOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleop(hardwareMap);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        hardwareMap.servo.get("armRightServo").setPosition(Constants.ARM_RIGHT_MIN);
        hardwareMap.servo.get("armLeftServo").setPosition(Constants.ARM_LEFT_MIN);
        hardwareMap.servo.get("clawServo").setPosition(Constants.CLAW_CLOSE);

        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d pose1 = coordinatesConvert(Pose1);
        Pose2d pose2 = coordinatesConvert(Pose2);

        drive.setPoseEstimate(startPose);



        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            telemetry.update();


            sleep(500);

            robot.liftLeft.setTargetPosition(Constants.LIFT_TARGET_POSITION);
            robot.liftRight.setTargetPosition(Constants.LIFT_TARGET_POSITION);

            robot.liftLeft.setPower(1.0);
            robot.liftRight.setPower(1.0);

            while (robot.liftLeft.isBusy() && robot.liftRight.isBusy() && opModeIsActive()) {
                telemetry.addData("Lift Left Pos", robot.liftLeft.getCurrentPosition());
                telemetry.addData("Lift Right Pos", robot.liftRight.getCurrentPosition());
                telemetry.update();
            }

          //  robot.liftLeft.setPower(0);
           // robot.liftRight.setPower(0);

            sleep(500);
            robot.armRight.setPosition(Constants.ARM_RIGHT_MAX);
            robot.armLeft.setPosition(Constants.ARM_LEFT_MAX);

            sleep(500);

            TrajectorySequence trajectoryToPose1 = drive.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(pose1.getX(), pose1.getY()))
                    .build();

            drive.followTrajectorySequence(trajectoryToPose1);

            sleep(500);
            hardwareMap.servo.get("clawServo").setPosition(Constants.CLAW_OPEN);

            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
