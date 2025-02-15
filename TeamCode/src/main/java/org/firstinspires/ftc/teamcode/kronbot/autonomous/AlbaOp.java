package org.firstinspires.ftc.teamcode.kronbot.autonomous;


import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_CLOSED;
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
import org.firstinspires.ftc.teamcode.kronbot.utils.autonomous.AutonomousConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Alba", group = Constants.MAIN_GROUP)
public class AlbaOp extends LinearOpMode {
    private final KronBot robot = new KronBot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleop(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //init positions
        hardwareMap.servo.get("intakeSlideLeft").setPosition(SLIDE_LEFT_CLOSED);
        hardwareMap.servo.get("intakeSlideRight").setPosition(SLIDE_RIGHT_CLOSED);

        hardwareMap.servo.get("armRightServo").setPosition(ARM_RIGHT_MIN);
        hardwareMap.servo.get("armLeftServo").setPosition(ARM_LEFT_MIN);

        hardwareMap.servo.get("intakeLeft").setPosition(INTAKE_LEFT_MIN);
        hardwareMap.servo.get("intakeRight").setPosition(INTAKE_RIGHT_MIN);

        hardwareMap.servo.get("clawServo").setPosition(CLAW_CLOSE);
        hardwareMap.servo.get("intakeServo").setPosition(Constants.INTAKE_CLAW_OPEN);

        //poses
        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d pose1=coordinatesConvert(Pose1); //go to basket
        Pose2d pose2=coordinatesConvert(Pose2); //reach saple
        Pose2d pose3=coordinatesConvert(Pose3); //go to basket again

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajectoryToPose1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(pose1.getX(), pose1.getY()))
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            //go to basket
            drive.followTrajectorySequence(trajectoryToPose1);

           //drop sample in the basket
            sleep(500);
            robot.liftLeft.setPower(1.0);
            robot.liftRight.setPower(1.0);

            robot.liftLeft.setTargetPosition(2200);
            robot.liftRight.setTargetPosition(2200);

            hardwareMap.servo.get("armLeftServo").setPosition(ARM_LEFT_MAX);
            hardwareMap.servo.get("armRightServo").setPosition(ARM_RIGHT_MAX);

            TrajectorySequence trajectoryToPose2 = drive.trajectorySequenceBuilder(pose1)
                    .lineTo(new Vector2d(pose2.getX(), pose2.getY()))  // Move to Pose 2
                    .turn(Math.toRadians(45))  // Rotate 45 degrees
                    .build();

            drive.followTrajectorySequence(trajectoryToPose2);


            sleep(1500);
            hardwareMap.servo.get("clawServo").setPosition(CLAW_OPEN);

            sleep(1500);

            //retract lift and arm
            hardwareMap.servo.get("clawServo").setPosition(CLAW_CLOSE);
            hardwareMap.servo.get("armLeftServo").setPosition(ARM_LEFT_MIN);
            hardwareMap.servo.get("armRightServo").setPosition(ARM_RIGHT_MIN);
            sleep(100);
            robot.liftLeft.setPower(1.0);
            robot.liftRight.setPower(1.0);
            robot.liftLeft.setTargetPosition(0);
            robot.liftRight.setTargetPosition(0);

        }

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
