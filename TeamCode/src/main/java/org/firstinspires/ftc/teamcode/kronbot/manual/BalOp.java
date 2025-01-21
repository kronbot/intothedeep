package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

@TeleOp(name = "Bal Driving", group = Constants.MAIN_GROUP)
public class BalOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    RobotCentricDrive robotCentricDrive;

    Gamepad drivingGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initSimpleDriving(hardwareMap);
        robot.initLift(hardwareMap);

        drivingGamepad = gamepad1;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);

        Button reverseButton = new Button();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            reverseButton.updateButton(drivingGamepad.circle);
            reverseButton.shortPress();
            robotCentricDrive.setReverse(reverseButton.getShortToggle());

            robotCentricDrive.run();

            //Lift
            robot.lift.run(drivingGamepad.right_trigger - drivingGamepad.left_trigger);
            telemetry.update();
        }
    }
}

