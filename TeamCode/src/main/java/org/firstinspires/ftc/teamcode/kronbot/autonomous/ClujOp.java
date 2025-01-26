package org.firstinspires.ftc.teamcode.kronbot.autonomous;


import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_INT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.coordinatesConvert;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.TrajectoryFactory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Cluj", group = Constants.MAIN_GROUP)
public class ClujOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        hardwareMap.servo.get("armRightServo").setPosition(Constants.ARM_RIGHT_MAX);
        hardwareMap.servo.get("clawServo").setPosition(Constants.CLAW_CLOSE);

        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d pose1=coordinatesConvert(Pose1);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectoryToPose1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(pose1.getX(), pose1.getY()))
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Following trajectory to pose1...");
            telemetry.update();

            drive.followTrajectorySequence(trajectoryToPose1);
            sleep(500);
            hardwareMap.servo.get("armRightServo").setPosition(Constants.ARM_RIGHT_INT);

            sleep(500);
            hardwareMap.servo.get("armRightServo").setPosition(Constants.ARM_RIGHT_MIN);
            sleep(500);
            hardwareMap.servo.get("clawServo").setPosition(Constants.CLAW_OPEN);



            telemetry.addData("Status", "Reached pose1.");
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
