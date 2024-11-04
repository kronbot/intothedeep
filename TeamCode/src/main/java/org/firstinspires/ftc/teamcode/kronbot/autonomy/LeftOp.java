package org.firstinspires.ftc.teamcode.kronbot.autonomy;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.AutonomousConstants;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.TrajectoryFactory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Left", group = Constants.MAIN_GROUP)
public class LeftOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAutonomy(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        List<TrajectorySequence> trajectory = TrajectoryFactory.createTrajectory(drive, robot, telemetry, true, () -> {sleep(1000); return; });

        waitForStart();

        drive.followTrajectorySequence(trajectory.get(0));
        sleep(AutonomousConstants.SLEEP);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}