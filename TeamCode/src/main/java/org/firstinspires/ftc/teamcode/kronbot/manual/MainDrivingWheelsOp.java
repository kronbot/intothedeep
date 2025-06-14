package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_UP;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_UP;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_ACTION_POSITION;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_INIT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_OPENED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_INIT;
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
@TeleOp(name = "Main Driving Wheels", group = Constants.MAIN_GROUP)
public class MainDrivingWheelsOp extends LinearOpMode {
    AtomicBoolean waitingRetraction = new AtomicBoolean(false);
    boolean extended = false;
    private final KronBot robot = new KronBot();

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    /// CONTROL SETUP
    Gamepad drivingGamepad;
    Gamepad utilityGamepad;
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

    Button specimenButton = new Button();
    double increment = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initTeleop(hardwareMap);

        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;

        robotCentricDrive = new RobotCentricDrive(robot, gamepad1);
        fieldCentricDrive = new FieldCentricDrive(robot, gamepad1);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        if (isStopRequested()) return;

        INITPose();
        sleep(100);

        while (opModeIsActive() && !isStopRequested()) {
            updateInput();
            // Wheels
            drive();

            if (waitingRetraction.get())
                continue;

            if (!waitingRetraction.get()) {
                //Lift
                handleLift();

                if (extended)
                    handleIntake();

                handleClaw();

                if (extensionButton.getShortToggle() && !extended)
                    extend();

            }

            // Retraction
            if (retractButton.getShortToggle() && !waitingRetraction.get() && extended)
                retract();
            else if (retractButton.getShortToggle() && waitingRetraction.get())
                retractButton.resetToggles();
            else if (extensionButton.getShortToggle() && waitingRetraction.get())
                extensionButton.resetToggles();

            telemetry.update();

            //manual slide
            if (extended)
                slide();
        }
    }

    private void slide() {
        if (utilityGamepad.right_bumper && robot.intakeSlideServoLeft.getPosition() < SLIDE_LEFT_OPENED)
            robot.intakeSlideServoLeft.setPosition(robot.intakeSlideServoLeft.getPosition() + increment);
        else if (utilityGamepad.left_bumper && robot.intakeSlideServoLeft.getPosition() > SLIDE_LEFT_CLOSED)
            robot.intakeSlideServoLeft.setPosition(robot.intakeSlideServoLeft.getPosition() - increment);
        if (utilityGamepad.left_bumper && robot.intakeSlideServoRight.getPosition() < SLIDE_RIGHT_OPENED)
            robot.intakeSlideServoRight.setPosition(robot.intakeSlideServoRight.getPosition() + increment);
        else if (utilityGamepad.right_bumper && robot.intakeSlideServoRight.getPosition() > SLIDE_LEFT_CLOSED)
            robot.intakeSlideServoRight.setPosition(robot.intakeSlideServoRight.getPosition() - increment);
    }

    private void INITPose() {
        robot.claw.setPosition(CLAW_OPEN);

        robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_INIT);
        robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_INIT);

        robot.armRight.setPosition(ARM_RIGHT_MIN);
        robot.armLeft.setPosition(ARM_LEFT_MIN);

        robot.intakeServoLeft.setPosition(INTAKE_LEFT_MAX);
        robot.intakeServoRight.setPosition(INTAKE_RIGHT_MAX);

    }

    private void updateInput() {
        //Update stick input

        //Drive
        driveModeButton.updateButton(drivingGamepad.triangle);
        driveModeButton.longPress();
        reverseButton.updateButton(drivingGamepad.cross);
        reverseButton.shortPress();

        //Arm Claw
        clawButton.updateButton(utilityGamepad.dpad_right);
        clawButton.shortPress();

        // Extension
        extensionButton.updateButton(utilityGamepad.circle);
        extensionButton.shortPress();

        // Retraction
        retractButton.updateButton(utilityGamepad.cross);
        retractButton.shortPress();
    }

    private void drive() {
        robotCentricDrive.setReverse(reverseButton.getShortToggle());
        if (!driveModeButton.getLongToggle()) robotCentricDrive.run();
        else fieldCentricDrive.run();
    }

    private void extend() {
        extended = true;
        robot.claw.setPosition(CLAW_OPEN);
        sleep(500);
        telemetry.addData("Claw", clawButton.getShortToggle() ? "OPEN" : "CLOSED");


        new Thread(() -> {
            extended=true;
            //robot.claw.setPosition(CLAW_OPEN);
            robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_OPENED);
            robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_OPENED);

            robot.intakeServoRight.setPosition(INTAKE_RIGHT_MAX);
            robot.intakeServoLeft.setPosition(INTAKE_LEFT_MAX);

            //extended=false;
        }).start();
            extended=false;
    }

    private void retract() {
        extended = false;

        new Thread(() -> {
            waitingRetraction.set(true);

            retractButton.resetToggles();
            extensionButton.resetToggles();

            robot.intakeWheelsRight.runContinuous(true, false);
            robot.intakeWheelsLeft.runContinuous(false, true);

            robot.claw.setPosition(CLAW_OPEN);
            sleep(500);

            armButton.resetToggles();
            robot.armRight.setPosition(ARM_RIGHT_MIN);
            robot.armLeft.setPosition(ARM_LEFT_MIN);

            robot.intakeServoRight.setPosition(INTAKE_RIGHT_UP);
            robot.intakeServoLeft.setPosition(INTAKE_LEFT_UP);

            robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_CLOSED);
            robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_CLOSED);


            try {
                telemetry.update();

                robot.claw.setPosition(CLAW_OPEN);
                Thread.sleep(1000);

                robot.claw.setPosition(CLAW_CLOSE);
                Thread.sleep(500);

                robot.intakeWheelsRight.runContinuous(false, false);
                robot.intakeWheelsLeft.runContinuous(false, false);

                waitingRetraction.set(false);
            } catch (InterruptedException e) {
                e.printStackTrace();

                waitingRetraction.set(false);
            }
        }).start();
    }

    private void handleLift() {
        robot.liftLeft.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
        robot.liftRight.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
        robot.liftLeft.getCurrentPosition();


        //Lift arm
        if (robot.liftLeft.getCurrentPosition() > LIFT_ACTION_POSITION) {
            armButton.resetToggles();
            robot.armRight.setPosition(ARM_RIGHT_MAX);
            robot.armLeft.setPosition(ARM_LEFT_MAX);
        } else {
            armButton.updateButton(utilityGamepad.dpad_down);
            armButton.shortPress();
            if (armButton.getShortToggle()) {
                robot.armRight.setPosition(ARM_RIGHT_MAX);
                robot.armLeft.setPosition(ARM_LEFT_MAX);
            } else {
                robot.armRight.setPosition(ARM_RIGHT_MIN);
                robot.armLeft.setPosition(ARM_LEFT_MIN);
            }
        }
    }

    private void handleIntake() {
        // Intake Wheels
        if (utilityGamepad.square) {
            robot.intakeWheelsRight.runContinuous(false, utilityGamepad.square);
            robot.intakeWheelsLeft.runContinuous(utilityGamepad.square, false);
        } else if (extended) {
            robot.intakeWheelsRight.runContinuous(true, false);
            robot.intakeWheelsLeft.runContinuous(false, true);
        } else {
            robot.intakeWheelsRight.runContinuous(false, false);
            robot.intakeWheelsLeft.runContinuous(false, false);
        }
    }

    private void handleClaw() {
            if (!extended && !waitingRetraction.get()) {
                if (clawButton.getShortToggle())
                    robot.claw.setPosition(CLAW_OPEN);
                else robot.claw.setPosition(CLAW_CLOSE);
            }
    }

}
