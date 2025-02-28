//package org.firstinspires.ftc.teamcode.kronbot.manual;
//
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MAX;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_TEST;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_OPEN;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MAX;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_UP;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LIFT_DOWN;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LIFT_UP;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MAX;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_UP;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_ROTATE_DEGREE;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_ROTATE_LEFT;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_ROTATE_RIGHT;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.LIFT_ACTION_POSITION;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ROTATION_ADDITION;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ROTATION_DIVISION;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_OPENED;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_SEMI_CLOSED;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_OPENED;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_CLOSED;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_CLOSED;
//import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_SEMI_CLOSED;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.teamcode.kronbot.KronBot;
//import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
//import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
//import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
//import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;
//
//import java.util.concurrent.atomic.AtomicBoolean;
//
////to do: button pulldown
//
///**
// * The main TeleOP program for the driving period of the game.
// *
// * @version 1.0
// */
//@TeleOp(name = "Main Driving Gecko", group = Constants.MAIN_GROUP)
//public class MainDrivingGeckoOp extends LinearOpMode {
//    AtomicBoolean waitingRetraction = new AtomicBoolean(false);
//    boolean extended = false;
//    private final KronBot robot = new KronBot();
//
//    RobotCentricDrive robotCentricDrive;
//    FieldCentricDrive fieldCentricDrive;
//
//    /// CONTROL SETUP
//    Gamepad drivingGamepad;
//    Gamepad utilityGamepad;
//    // Wheels
//    Button driveModeButton = new Button();
//    Button reverseButton = new Button();
//    // Claw
//    Button clawButton = new Button();
//    //Intake Lift
//    Button intakeButton = new Button();
//    //Intake Rotation
//    Button intakeRotationButton = new Button();
//    // Arm
//    Button armButton = new Button();
//    Button armButtontest = new Button();
//    // Actions
//    Button retractButton = new Button();
//    Button extensionButton = new Button();
//    double xLeft = 0, yLeft = 0, xRight = 0, yRight = 0;
//    double increment = 0.01;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.initTeleop(hardwareMap);
//
//        drivingGamepad = gamepad1;
//        utilityGamepad = gamepad2;
//
//        robotCentricDrive = new RobotCentricDrive(robot, gamepad1);
//        fieldCentricDrive = new FieldCentricDrive(robot, gamepad1);
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            telemetry.addLine("Initialization Ready");
//            telemetry.update();
//        }
//
//        if (isStopRequested()) return;
//
//        INITPose();
//        sleep(100);
//
//        while (opModeIsActive() && !isStopRequested()) {
//            updateInput();
//
//            // Wheels
//            Drive();
//
//            if (waitingRetraction.get())
//                continue;
//
//            if (!waitingRetraction.get()) {
//                //Lift
//                handleLift();
//
//                if (extended)
//                    handleIntake();
//
//                //Claw
//                handleClaw();
//
//                // Extension
//                extensionButton.updateButton(utilityGamepad.circle);
//                extensionButton.shortPress();
//                if (extensionButton.getShortToggle() && !extended)
//                    extend();
//
//            }
//
//            // Retraction
//            retractButton.updateButton(utilityGamepad.cross);
//            retractButton.shortPress();
//            if (retractButton.getShortToggle() && !waitingRetraction.get() && extended)
//                retract();
//            else if (retractButton.getShortToggle() && waitingRetraction.get())
//                retractButton.resetToggles();
//            else if (extensionButton.getShortToggle() && waitingRetraction.get())
//                extensionButton.resetToggles();
//
//            telemetry.update();
//        }
//    }
//
//    private void Drive() {
//        driveModeButton.updateButton(drivingGamepad.triangle);
//        driveModeButton.longPress();
//
//        reverseButton.updateButton(drivingGamepad.cross);
//        reverseButton.shortPress();
//        robotCentricDrive.setReverse(reverseButton.getShortToggle());
//        if (!driveModeButton.getLongToggle()) robotCentricDrive.run();
//        else fieldCentricDrive.run();
//    }
//
//    private void INITPose() {
//        robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_CLOSED);
//        robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_CLOSED);
//        robot.intakeClawServo.setPosition(CLAW_OPEN);
//        robot.intakeServoRight.setPosition(INTAKE_RIGHT_MAX);
//    }
//
//    private void updateInput() {
//        //Stick update
//        xLeft = gamepad2.left_stick_x;
//        yLeft = gamepad2.left_stick_y;
//
//        xRight = gamepad2.right_stick_x;
//        yRight = gamepad2.right_stick_y;
//
//        //Drive
//        driveModeButton.updateButton(drivingGamepad.triangle);
//        driveModeButton.longPress();
//
//        reverseButton.updateButton(drivingGamepad.cross);
//        reverseButton.shortPress();
//
//        //Intake claw
//        clawButton.updateButton(utilityGamepad.dpad_right);
//        clawButton.shortPress();
//
//        //Intake Height
//        intakeButton.updateButton(utilityGamepad.dpad_up);
//        intakeButton.shortPress();
//
//        // Extension
//        extensionButton.updateButton(utilityGamepad.circle);
//        extensionButton.shortPress();
//
//        // Retraction
//        retractButton.updateButton(utilityGamepad.cross);
//        retractButton.shortPress();
//    }
//
//    private void extend() {
//        extended = true;
//
//        intakeButton.resetToggles();
//
//        robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_OPENED);
//        robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_OPENED);
//
//        robot.intakeServoRight.setPosition(INTAKE_RIGHT_MAX);
//        robot.intakeServoLeft.setPosition(INTAKE_LEFT_MAX);
//
//
//        robot.claw.setPosition(CLAW_OPEN);
//    }
//
//    private void retract() {
//        extended = false;
//
//        new Thread(() -> {
//            waitingRetraction.set(true);
//
//            retractButton.resetToggles();
//            extensionButton.resetToggles();
//
//            armButton.resetToggles();
//            robot.armRight.setPosition(ARM_RIGHT_MIN);
//            robot.armLeft.setPosition(ARM_LEFT_MIN);
//
//            robot.intakeServoRight.setPosition(INTAKE_RIGHT_UP);
//            robot.intakeServoLeft.setPosition(INTAKE_LEFT_UP);
//
//
//            try {
//                Thread.sleep(1000);
//
//                robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_SEMI_CLOSED);
//                robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_SEMI_CLOSED);
//                Thread.sleep(500);
//                intakeRotationButton.resetToggles();
//                robot.intakeRotateServo.setPosition(INTAKE_ROTATE_DEGREE);
//
//                Thread.sleep(500);
//
//                //robot.claw.setPosition(CLAW_CLOSE);
//                telemetry.addData("Retraction", "Claw closing...");
//
//                Thread.sleep(500);
//
//                intakeButton.resetToggles();
//                robot.intakeLiftServo.setPosition(INTAKE_LIFT_UP);
//
//                waitingRetraction.set(false);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//
//                waitingRetraction.set(false);
//            }
//        }).start();
//    }
//
//    private void handleLift() {
//        robot.liftLeft.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
//        robot.liftRight.run(utilityGamepad.right_trigger - utilityGamepad.left_trigger);
//
//        //Lift arm
//        if (robot.liftLeft.getCurrentPosition() > LIFT_ACTION_POSITION) {
//            armButton.resetToggles();
//            robot.armRight.setPosition(ARM_RIGHT_MAX);
//            robot.armLeft.setPosition(ARM_LEFT_MAX);
//        } else {
//            armButton.updateButton(utilityGamepad.dpad_down);
//            armButton.shortPress();
//            if (armButton.getShortToggle()) {
//                robot.armRight.setPosition(ARM_TEST);
//                robot.armLeft.setPosition(ARM_LEFT_MAX);
//            } else {
//                robot.armRight.setPosition(ARM_RIGHT_MIN);
//                robot.armLeft.setPosition(ARM_LEFT_MIN);
//                robot.claw.setPosition(CLAW_CLOSE);
//            }
//        }
//    }
//
//    private void handleIntake() {
//        //Intake height
//        robot.intakeLiftServo.setPosition(intakeButton.getShortToggle() ? INTAKE_LIFT_DOWN : INTAKE_LIFT_UP);
//
//        //Intake Rotation
//         if (Math.hypot(xRight, yRight) > 0.1) {
//            double angle = (Math.atan2(-yRight, xRight) + ROTATION_ADDITION) / ROTATION_DIVISION;
//            robot.intakeRotateServo.setPosition(
//                    Math.max(INTAKE_ROTATE_LEFT, Math.min(angle, INTAKE_ROTATE_RIGHT))
//            );
//        }
//    }
//
//    private void handleClaw() {
//        if (!waitingRetraction.get()) {
//            clawButton.updateButton(utilityGamepad.dpad_right);
//            clawButton.shortPress();
//            if (!extended && !waitingRetraction.get()) {
//                if (clawButton.getShortToggle())
//                    robot.claw.setPosition(CLAW_OPEN);
//                else robot.claw.setPosition(CLAW_CLOSE);
//            }
//        }
//    }
//}