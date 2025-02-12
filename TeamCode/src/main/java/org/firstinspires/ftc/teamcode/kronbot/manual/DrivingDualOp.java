package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_OPENED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_OPENED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_CLOSED;

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
@TeleOp(name = "Dual Driving", group = Constants.MAIN_GROUP)
public class DrivingDualOp extends LinearOpMode {
    private final KronBot robot = new KronBot();

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    Gamepad drivingGamepad;
    Gamepad utilityGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        AtomicBoolean waitingRetraction = new AtomicBoolean(false);
        boolean extended = false;

        robot.initTeleop(hardwareMap);

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, gamepad1);
        fieldCentricDrive = new FieldCentricDrive(robot, gamepad1);
        // Wheels
        Button driveModeButton = new Button();
        Button reverseButton = new Button();

        // Claw
        Button clawButton = new Button();

        //Intake
        Button intakeButton = new Button();

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
                robot.liftLeft.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
                robot.liftRight.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
                //when the lift reaches a certain position, the arm lifts ???
                if (robot.liftLeft.getCurrentPosition() > 1500) {
                    robot.armRight.setPosition(ARM_RIGHT_MAX);
                    robot.armLeft.setPosition(ARM_LEFT_MAX);
                } else {
                    robot.armRight.setPosition(ARM_RIGHT_MIN);
                    robot.armLeft.setPosition(ARM_LEFT_MIN);
                }

                //Intake
                intakeButton.updateButton(utilityGamepad.square);
                intakeButton.shortPress();
                if (intakeButton.getShortToggle())
                    robot.intakeClawServo.setPosition(INTAKE_CLOSE);
                else robot.intakeClawServo.setPosition(INTAKE_OPEN);

                // Claw
                clawButton.updateButton(utilityGamepad.dpad_right);
                clawButton.shortPress();
                //when the robot is retracted and the claw closes, the intake opens ???
                if (clawButton.getShortToggle() && !extended) {
                    robot.claw.setPosition(CLAW_CLOSE);
                    sleep(100);
                    robot.intakeClawServo.setPosition(INTAKE_OPEN);
                } //if it's not retracted it works normally
                else if (clawButton.getShortToggle() && extended) {
                    robot.claw.setPosition(CLAW_CLOSE);
                }
                else robot.claw.setPosition(CLAW_OPEN);

                // Arm
                armButton.updateButton(utilityGamepad.dpad_up);
                armButton.shortPress();
                if (armButton.getShortToggle()) {
                    robot.armRight.setPosition(ARM_RIGHT_MAX);
                    robot.armLeft.setPosition(ARM_LEFT_MAX);
                } else {
                    robot.armRight.setPosition(ARM_RIGHT_MIN);
                    robot.armLeft.setPosition(ARM_LEFT_MIN);
                }

                // Extension
                extensionButton.updateButton(utilityGamepad.circle);
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
            retractButton.updateButton(utilityGamepad.cross);
            retractButton.shortPress();
            if (retractButton.getShortToggle() && !waitingRetraction.get() && extended) {
                extended = false;

                new Thread(() -> {
                    waitingRetraction.set(true);

                    retractButton.resetToggles();
                    extensionButton.resetToggles();

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

                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }

                    waitingRetraction.set(false);
                }).start();
            } else if (retractButton.getShortToggle() && waitingRetraction.get()) retractButton.resetToggles();
            else if (extensionButton.getShortToggle() && waitingRetraction.get()) extensionButton.resetToggles();

            // Wheels
            driveModeButton.updateButton(drivingGamepad.triangle);
            driveModeButton.longPress();

            reverseButton.updateButton(drivingGamepad.cross);
            reverseButton.shortPress();
            robotCentricDrive.setReverse(reverseButton.getShortToggle());
            if (!driveModeButton.getLongToggle()) robotCentricDrive.run();
            else fieldCentricDrive.run();

            telemetry.update();
        }
    }
}
