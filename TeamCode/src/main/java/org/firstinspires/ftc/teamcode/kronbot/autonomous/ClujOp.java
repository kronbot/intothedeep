package org.firstinspires.ftc.teamcode.kronbot.autonomous;


import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.Pose1;
import static org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants.coordinatesConvert;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Cluj", group = Constants.MAIN_GROUP)
public class ClujOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize motors
        DcMotor liftLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        DcMotor liftRight = hardwareMap.get(DcMotor.class, "liftMotorRight");
//
//        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardwareMap.servo.get("armRightServo").setPosition(Constants.ARM_RIGHT_MIN);
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
            telemetry.update();

            //drive.followTrajectorySequence(trajectoryToPose1);
            sleep(500);
            liftLeft.setTargetPosition(Constants.LIFT_TARGET_POSITION);
            liftRight.setTargetPosition(Constants.LIFT_TARGET_POSITION);

            liftLeft.setPower(1.0);
            liftRight.setPower(1.0);
            // Wait until lift reaches position
            while (liftLeft.isBusy() && liftRight.isBusy() && opModeIsActive()) {
                telemetry.addData("Lift Left Pos", liftLeft.getCurrentPosition());
                telemetry.addData("Lift Right Pos", liftRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop the lift motors
            liftLeft.setPower(0);
            liftRight.setPower(0);

            sleep(500);
            hardwareMap.servo.get("armRightServo").setPosition(Constants.ARM_RIGHT_MAX);


            sleep(500);
            hardwareMap.servo.get("clawServo").setPosition(Constants.CLAW_OPEN);


            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
