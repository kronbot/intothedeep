package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_OPENED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_OPENED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * The main TeleOP program for the driving period of the game
 *
 * @version 1.0
 */
@TeleOp(name = "Single Driving", group = Constants.MAIN_GROUP)
public class DrivingSingleOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    Gamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        AtomicBoolean waitingRetraction = new AtomicBoolean(false);
        boolean extended = false;

        robot.initTeleop(hardwareMap);

        gamepad = gamepad1;

        robotCentricDrive = new RobotCentricDrive(robot, gamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, gamepad);

        // Wheels
        Button driveModeButton = new Button();
        Button reverseButton = new Button();

        // Claw
        Button clawButton = new Button();

        // Arm
        Button armButton = new Button();

        // Actions
        Button retractButton = new Button();
        Button extensionButton = new Button();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_CLOSED);
        robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_CLOSED);

        robot.armRight.setPosition(ARM_RIGHT_MIN);
        robot.armLeft.setPosition(ARM_LEFT_MIN);

        while (opModeIsActive() && !isStopRequested()) {
            if (!waitingRetraction.get()) {
                // Lift
//                robot.liftLeft.run(gamepad.right_trigger - gamepad.left_trigger);
//                robot.liftRight.run(gamepad.right_trigger - gamepad.left_trigger);
                double liftPower = gamepad.right_trigger - gamepad.left_trigger;
                int leftPosition = robot.liftLeft.getCurrentPosition();
                int rightPosition = robot.liftRight.getCurrentPosition();

                // Prevent going below the initial position
                if (leftPosition <= Constants.LIFT_INIT_POSITION && liftPower < 0) {
                    liftPower = 0;
                }
                if (rightPosition <= Constants.LIFT_INIT_POSITION && liftPower < 0) {
                    liftPower = 0;
                }

                robot.liftLeft.run(liftPower);
                robot.liftRight.run(liftPower);
                telemetry.addData("Left", robot.liftLeft.getCurrentPosition());
                telemetry.addData("Right", robot.liftRight.getCurrentPosition());

                // Intake Wheels
                if (gamepad.dpad_down) {
                    robot.intakeWheelsRight.runContinuous(false, gamepad.dpad_down);
                    robot.intakeWheelsLeft.runContinuous(gamepad.dpad_down, false);
                } else if (extended) {
                    robot.intakeWheelsRight.runContinuous(true, false);
                    robot.intakeWheelsLeft.runContinuous(false, true);
                } else {
                    robot.intakeWheelsRight.runContinuous(false, false);
                    robot.intakeWheelsLeft.runContinuous(false, false);
                }

                // Claw
                clawButton.updateButton(gamepad.dpad_right);
                clawButton.shortPress();
                if (clawButton.getShortToggle())
                    robot.claw.setPosition(CLAW_CLOSE);
                else robot.claw.setPosition(CLAW_OPEN);

                // Arm
                armButton.updateButton(gamepad.dpad_up);
                armButton.shortPress();
                if (armButton.getShortToggle()) {
                    robot.armRight.setPosition(ARM_RIGHT_MAX);
                    robot.armLeft.setPosition(ARM_LEFT_MAX);
                } else {
                    robot.armRight.setPosition(ARM_RIGHT_MIN);
                    robot.armLeft.setPosition(ARM_LEFT_MIN);
                }

                // Extension
                extensionButton.updateButton(gamepad.circle);
                extensionButton.shortPress();
                if (extensionButton.getShortToggle() && !extended) {
                    extended = true;

                    robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_OPENED);
                    robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_OPENED);

                    robot.intakeServoRight.setPosition(INTAKE_RIGHT_MAX);
                    robot.intakeServoLeft.setPosition(INTAKE_LEFT_MAX);
                }
            }

            // Retraction
            retractButton.updateButton(gamepad.square);
            retractButton.shortPress();
            if (retractButton.getShortToggle() && !waitingRetraction.get() && extended) {
                extended = false;

                new Thread(() -> {
                    waitingRetraction.set(true);

                    retractButton.resetToggles();
                    extensionButton.resetToggles();

                    robot.intakeWheelsRight.runContinuous(true, false);
                    robot.intakeWheelsLeft.runContinuous(false, true);

                    armButton.resetToggles();
                    robot.armRight.setPosition(ARM_RIGHT_MIN);
                    robot.armLeft.setPosition(ARM_LEFT_MIN);

                    robot.intakeServoRight.setPosition(INTAKE_RIGHT_MIN);
                    robot.intakeServoLeft.setPosition(INTAKE_LEFT_MIN);

                    robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_CLOSED);
                    robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_CLOSED);

                    clawButton.resetToggles();
                    robot.claw.setPosition(CLAW_OPEN);


                    try {
                        Thread.sleep(900);
//                        robot.claw.setPosition(CLAW_CLOSE);
//                        clawButton.simulateShortPress();

                        robot.intakeWheelsRight.runContinuous(false, false);
                        robot.intakeWheelsLeft.runContinuous(false, false);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    waitingRetraction.set(false);
                }).start();
            } else if (retractButton.getShortToggle() && waitingRetraction.get()) retractButton.resetToggles();
            else if (extensionButton.getShortToggle() && waitingRetraction.get()) extensionButton.resetToggles();

            // Wheels
            driveModeButton.updateButton(gamepad.triangle);
            driveModeButton.longPress();

            reverseButton.updateButton(gamepad.cross);
            reverseButton.shortPress();
            robotCentricDrive.setReverse(reverseButton.getShortToggle());
            if (!driveModeButton.getLongToggle()) robotCentricDrive.run();
            else fieldCentricDrive.run();

            telemetry.update();
        }
    }
}
