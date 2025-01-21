package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.WHEELS_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.WHEELS_MIN;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

/**
 * The main TeleOP program for the driving period of the game.
 *
 * @version 1.0
 */
@TeleOp(name = "Main Driving", group = Constants.MAIN_GROUP)
public class MainDrivingOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    Gamepad drivingGamepad;
    Gamepad utilityGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleop(hardwareMap);

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);

        Button driveModeButton = new Button();
        Button reverseButton = new Button();

        Button clawButton = new Button();
        Button handButton = new Button();

        Button intakeWheelsButton = new Button();

        Servo intakeServo = new Servo(hardwareMap);

        Button intakePassButton = new Button();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Lift
            robot.lift.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);

            //intake sliders
            intakeServo.setPosition(INTAKE_MIN);
            intakeServo.runIncrement(utilityGamepad.right_bumper, utilityGamepad.left_bumper);

            //intake wheels
            intakeWheelsButton.updateButton(utilityGamepad.triangle);
            intakeWheelsButton.shortPress();
            robot.intakeWheels.run(intakeWheelsButton.getShortToggle());

            //intake pass
            intakePassButton.updateButton(utilityGamepad.square);
            intakePassButton.shortPress();
            robot.intakePass.run(intakePassButton.getShortToggle());

            // Claw
            clawButton.updateButton(utilityGamepad.circle);
            clawButton.shortPress();
            robot.claw.run(clawButton.getShortToggle());

            // Hand
            handButton.updateButton(utilityGamepad.square);
            handButton.shortPress();
            robot.hand.run(handButton.getShortToggle());

            // Arm
            if (handButton.getShortToggle()) {
                robot.armLeft.setPosition(ARM_LEFT_MIN);
                robot.armRight.setPosition(ARM_RIGHT_MIN);
            } else {
                robot.armLeft.setPosition(ARM_LEFT_MAX);
                robot.armRight.setPosition(ARM_RIGHT_MAX);
            }

            // Wheels
            driveModeButton.updateButton(drivingGamepad.square);
            driveModeButton.longPress();

            reverseButton.updateButton(drivingGamepad.circle);
            reverseButton.shortPress();
            robotCentricDrive.setReverse(reverseButton.getShortToggle());
            if (!driveModeButton.getLongToggle()) {
                robotCentricDrive.run();
                robotCentricDrive.telemetry(telemetry);
            } else {
                fieldCentricDrive.run();
                fieldCentricDrive.telemetry(telemetry);
            }

            telemetry.update();
        }
    }
}
