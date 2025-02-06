package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_INT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_INT;
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
@TeleOp(name = "Driving Single", group = Constants.MAIN_GROUP)
public class DrivingSingleOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    Gamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        AtomicBoolean waitingRetraction = new AtomicBoolean(false);
        AtomicBoolean waitingExtension = new AtomicBoolean(false);

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
        Button leftButton = new Button();
        Button topButton = new Button();
        Button rightButton = new Button();

        // Actions
        Button retractButton = new Button();
        Button extensionButton = new Button();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if (!waitingRetraction.get() && !waitingExtension.get()) {
                // Lift
                robot.liftLeft.run(gamepad.right_trigger - gamepad.left_trigger);
                robot.liftRight.run(gamepad.right_trigger - gamepad.left_trigger);

                // Intake Wheels
                robot.intakeWheelsRight.runContinuous(gamepad.dpad_down, false);
                robot.intakeWheelsRight.runContinuous(gamepad.dpad_down, false);

                // Claw
                clawButton.updateButton(gamepad.circle);
                clawButton.shortPress();
                if (clawButton.getShortToggle())
                    robot.claw.setPosition(CLAW_OPEN);
                else robot.claw.setPosition(CLAW_CLOSE);

                // Arm
                leftButton.updateButton(gamepad.dpad_left);
                leftButton.shortPress();
                topButton.updateButton(gamepad.dpad_up);
                topButton.shortPress();
                rightButton.updateButton(gamepad.dpad_right);
                rightButton.shortPress();
                if (leftButton.getShortToggle()) {
                    robot.armRight.setPosition(ARM_RIGHT_MAX);
                    robot.armLeft.setPosition(ARM_LEFT_MAX);
                    leftButton.resetToggles();
                } else if (topButton.getShortToggle()) {
                    robot.armRight.setPosition(ARM_RIGHT_INT);
                    robot.armLeft.setPosition(ARM_LEFT_INT);
                    topButton.resetToggles();
                } else if (rightButton.getShortToggle()) {
                    robot.armRight.setPosition(ARM_RIGHT_MIN);
                    robot.armLeft.setPosition(ARM_LEFT_MIN);
                    rightButton.resetToggles();
                }
            }

            // Extension
            extensionButton.updateButton(gamepad.circle);
            extensionButton.shortPress();
            if (extensionButton.getShortToggle() && !waitingExtension.get()  && !waitingRetraction.get()) {
                new Thread(() -> {
                    waitingExtension.set(true);

                    extensionButton.resetToggles();

                    robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_OPENED);
                    robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_OPENED);

                    try {
                        Thread.sleep(350);
                        robot.intakeServoRight.setPosition(INTAKE_RIGHT_MAX);
                        robot.intakeServoLeft.setPosition(INTAKE_LEFT_MAX);

                        robot.intakeWheelsRight.runContinuous(true, false);
                        robot.intakeWheelsLeft.runContinuous(true, false);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    waitingExtension.set(false);
                });
            }

            // Retraction
            retractButton.updateButton(gamepad.square);
            retractButton.shortPress();
            if (retractButton.getShortToggle() && !waitingExtension.get()  && !waitingRetraction.get()) {
                new Thread(() -> {
                    waitingRetraction.set(true);

                    retractButton.resetToggles();

                    robot.intakeWheelsRight.runContinuous(false, false);
                    robot.intakeWheelsLeft.runContinuous(false, false);

                    clawButton.resetToggles();
                    robot.claw.setPosition(CLAW_OPEN);

                    try {
                        Thread.sleep(350);
                        robot.intakeServoRight.setPosition(INTAKE_RIGHT_MIN);
                        robot.intakeServoLeft.setPosition(INTAKE_LEFT_MIN);

                        robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_CLOSED);
                        robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_CLOSED);

                        clawButton.resetToggles();
                        robot.claw.setPosition(CLAW_OPEN);

                        Thread.sleep(350);
                        topButton.resetToggles();
                        leftButton.resetToggles();
                        rightButton.resetToggles();
                        robot.armRight.setPosition(ARM_RIGHT_MIN);
                        robot.armLeft.setPosition(ARM_LEFT_MIN);

                        Thread.sleep(800);
                        robot.intakeWheelsRight.runContinuous(false, true);
                        robot.intakeWheelsLeft.runContinuous(false, true);
                        Thread.sleep(0);

                        robot.claw.setPosition(CLAW_CLOSE);

                        Thread.sleep(300);
                        robot.intakeWheelsRight.runContinuous(false, false);
                        robot.intakeWheelsLeft.runContinuous(false, false);

                        robot.armRight.setPosition(ARM_RIGHT_INT);
                        robot.armLeft.setPosition(ARM_LEFT_INT);

                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    waitingRetraction.set(false);
                }).start();
            }

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
