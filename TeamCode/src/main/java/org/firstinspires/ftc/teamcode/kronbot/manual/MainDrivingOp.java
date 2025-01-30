package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_INT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_TEST;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_INT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_TEST;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CONTROLLER_DEADZONE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_REST_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_REVERSE_POWER;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_MAX_LEFT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_MAX_RIGHT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_MIN_LEFT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_MIN_RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

import java.util.concurrent.atomic.AtomicBoolean;

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

    double increment = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        AtomicBoolean waiting = new AtomicBoolean(false);

        robot.initTeleop(hardwareMap);

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);

        Button driveModeButton = new Button();
        Button reverseButton = new Button();

        Button clawButton = new Button();

        Button leftButton = new Button();
        Button topButton = new Button();
        Button rightButton = new Button();

        Button doAllButton = new Button();
        Button intakeButton = new Button();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Lift
//            robot.liftLeft.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
//            robot.liftRight.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
            if (utilityGamepad.right_trigger > CONTROLLER_DEADZONE) {
                robot.liftMotorLeft.setPower(-utilityGamepad.right_trigger);
                robot.liftMotorRight.setPower(-utilityGamepad.right_trigger);
            } else if (utilityGamepad.left_trigger > CONTROLLER_DEADZONE) {
                robot.liftMotorLeft.setPower(utilityGamepad.left_trigger * LIFT_REVERSE_POWER);
                robot.liftMotorRight.setPower(utilityGamepad.left_trigger * LIFT_REVERSE_POWER);
            } else {
                robot.liftMotorLeft.setPower(-LIFT_REST_POWER);
                robot.liftMotorRight.setPower(-LIFT_REST_POWER);
            }

            if (!waiting.get()) {
                // Intake Sliders
                if (utilityGamepad.right_bumper && robot.intakeSlideServoLeft.getPosition() < SLIDE_MAX_LEFT)
                    robot.intakeSlideServoLeft.setPosition(robot.intakeSlideServoLeft.getPosition() + increment);
                else if (utilityGamepad.left_bumper && robot.intakeSlideServoLeft.getPosition() > SLIDE_MIN_LEFT)
                    robot.intakeSlideServoLeft.setPosition(robot.intakeSlideServoLeft.getPosition() - increment);
                if (utilityGamepad.left_bumper && robot.intakeSlideServoRight.getPosition() < SLIDE_MAX_RIGHT)
                    robot.intakeSlideServoRight.setPosition(robot.intakeSlideServoRight.getPosition() + increment);
                else if (utilityGamepad.right_bumper && robot.intakeSlideServoRight.getPosition() > SLIDE_MIN_RIGHT)
                    robot.intakeSlideServoRight.setPosition(robot.intakeSlideServoRight.getPosition() - increment);

                //intake Wheels
                robot.intakeWheels.runContinuous(utilityGamepad.right_stick_y < -CONTROLLER_DEADZONE, utilityGamepad.right_stick_y > CONTROLLER_DEADZONE);

                // Intake Hand
                intakeButton.updateButton(utilityGamepad.triangle);
                intakeButton.shortPress();
                if (intakeButton.getShortToggle()) {
                    robot.intakeServoRight.setPosition(INTAKE_MIN);
                    robot.intakeServoLeft.setPosition(INTAKE_MIN);
                } else {
                    robot.intakeServoRight.setPosition(INTAKE_MAX);
                    robot.intakeServoLeft.setPosition(INTAKE_MAX);
                }

                // Claw
                clawButton.updateButton(utilityGamepad.circle);
                clawButton.shortPress();
                if (clawButton.getShortToggle())
                    robot.claw.setPosition(CLAW_OPEN);
                else robot.claw.setPosition(CLAW_CLOSE);

                // Arm
                leftButton.updateButton(utilityGamepad.dpad_left);
                leftButton.shortPress();
                topButton.updateButton(utilityGamepad.dpad_up);
                topButton.shortPress();
                rightButton.updateButton(utilityGamepad.dpad_right);
                rightButton.shortPress();
                if (leftButton.getShortToggle()) {
                    robot.armRight.setPosition(ARM_RIGHT_MIN);
                    robot.armLeft.setPosition(ARM_LEFT_MIN);
                    leftButton.resetToggles();
                } else if (topButton.getShortToggle()) {
                    robot.armRight.setPosition(ARM_RIGHT_INT);
                    robot.armLeft.setPosition(ARM_LEFT_INT);
                    topButton.resetToggles();
                } else if (rightButton.getShortToggle()) {
                    robot.armRight.setPosition(ARM_RIGHT_MAX);
                    robot.armLeft.setPosition(ARM_LEFT_MAX);
                    rightButton.resetToggles();
                }
            }

            // All
            doAllButton.updateButton(utilityGamepad.cross);
            doAllButton.shortPress();
            if (doAllButton.getShortToggle()) {
                new Thread(() -> {
                    waiting.set(true);

                    doAllButton.resetToggles();

                    clawButton.resetToggles();
                    robot.claw.setPosition(CLAW_OPEN);

                    topButton.resetToggles();
                    leftButton.resetToggles();
                    rightButton.resetToggles();
                    robot.armRight.setPosition(ARM_RIGHT_MAX);
                   // robot.armLeft.setPosition(ARM_LEFT_MAX);

                    try {
                        Thread.sleep(350);
                        intakeButton.resetToggles();
                        robot.intakeServoRight.setPosition(INTAKE_MAX);
                        robot.intakeSlideServoRight.setPosition(SLIDE_MAX_RIGHT);

                        Thread.sleep(800);
                        robot.intakeWheels.runContinuous(false, true);
                        Thread.sleep(300);

                        robot.claw.setPosition(CLAW_CLOSE);
                        Thread.sleep(300);
                        robot.armRight.setPosition(ARM_RIGHT_TEST);
                        //robot.armLeft.setPosition(ARM_LEFT_TEST);
                        Thread.sleep(300);
                        robot.armRight.setPosition(ARM_RIGHT_MAX);
                        //robot.armLeft.setPosition(ARM_LEFT_MAX);
                        Thread.sleep(300);
                        robot.armRight.setPosition(ARM_RIGHT_INT);
                        //robot.armLeft.setPosition(ARM_LEFT_INT);
                        robot.intakeWheels.runContinuous(false, false);

                        Thread.sleep(200);
                        waiting.set(false);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }).start();
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
