package org.firstinspires.ftc.teamcode.kronbot.tests;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_INIT_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.TEST_GROUP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;

@Autonomous(name = "Test Autonomy", group = TEST_GROUP)
public class TestAutonomy extends LinearOpMode {
    private final KronBot robot = new KronBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAutonomy(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }

        waitForStart();

        robot.lift.setTargetPosition(LIFT_MAX_POSITION / 2);
        robot.lift.setPower(LIFT_POWER);
        while (robot.lift.isBusy());
        sleep(500);
        robot.lift.setTargetPosition(LIFT_INIT_POSITION);
        robot.lift.setPower(LIFT_POWER / 3);
        while (robot.lift.isBusy());
        sleep(500);
        robot.lift.setTargetPosition(LIFT_MAX_POSITION / 2);
        robot.lift.setPower(LIFT_POWER);
        while (robot.lift.isBusy());
        sleep(500);
        robot.lift.setTargetPosition(LIFT_INIT_POSITION);
        robot.lift.setPower(LIFT_POWER / 3);
        while (robot.lift.isBusy());
        sleep(500);

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.update();
        }
    }
}
