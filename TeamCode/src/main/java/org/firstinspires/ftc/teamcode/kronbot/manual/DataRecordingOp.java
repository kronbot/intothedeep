package org.firstinspires.ftc.teamcode.kronbot.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.*;

@TeleOp(name = "DATA Recorder", group = MAIN_GROUP)
public class DataRecordingOp extends LinearOpMode {
    private static final long RECORD_INTERVAL_MS = 20; // 50Hz sampling
    private final KronBot robot = new KronBot();
    private SampleMecanumDrive drive;
    private FileWriter inputRecorder;
    private long startTime;

    // Robot state
    private boolean extended = false;
    AtomicBoolean waitingRetraction = new AtomicBoolean(false);

    // Drive components
    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;
    Gamepad controlGamepad;

    // Drive controls
    Button driveModeButton = new Button();
    Button reverseButton = new Button();

    // Mechanism controls
    Button clawButton = new Button();
    Button armButton = new Button();
    Button retractButton = new Button();
    Button extensionButton = new Button();

    // The 'increment' variable for manual slide control is only in the recorder,
    // and since the 'Single Driving' code doesn't have it, we'll remove manual slide control
    // if the goal is to align behavior. If you want to keep manual slide in recorder,
    // we can re-add it with appropriate keybinds, but it's not present in the reference code.
    // double increment = 0.01; // Removed as it's not in the reference 'Single Driving' code

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            // init all hardware
            robot.initTeleop(hardwareMap);
            drive = new SampleMecanumDrive(hardwareMap);
            drive.setPoseEstimate(new Pose2d());

            controlGamepad = gamepad1;
            robotCentricDrive = new RobotCentricDrive(robot, controlGamepad);
            fieldCentricDrive = new FieldCentricDrive(robot, controlGamepad);

            String filePath = Environment.getExternalStorageDirectory().getPath() + "/robot_inputs.csv";
            inputRecorder = new FileWriter(filePath);
            inputRecorder.write(
                    "Time,LeftRearPower,RightRearPower,LeftFrontPower,RightFrontPower," +
                            "X,Y,Heading,Voltage," +
                            "LiftLeftPos,LiftRightPos,SlideLeftPos,SlideRightPos,ClawPos," +
                            "ArmLeftPos,ArmRightPos,IntakeLeftPos,IntakeRightPos," +
                            "IntakeLeftPower,IntakeRightPower\n"
            );

            // Initialize robot pose
            INITPose();

            while (!isStopRequested() && !opModeIsActive()) {
                telemetry.addLine("Enhanced Single Gamepad Recorder Ready");
                telemetry.addLine("Controls:");
                telemetry.addLine("Left Stick: Drive");
                telemetry.addLine("Right Stick X: Turn");
                telemetry.addLine("Triangle (long): Field/Robot Centric");
                telemetry.addLine("Cross: Reverse Drive");
                telemetry.addLine("Circle: Extend");
                telemetry.addLine("Square: Retract");
                telemetry.addLine("R2/L2: Lift Control");
                telemetry.addLine("DPad Up: Arm Toggle");
                telemetry.addLine("DPad Right: Claw Toggle");
                telemetry.addData("File Path", filePath);
                telemetry.addData("Rate (Hz)", 1000.0 / RECORD_INTERVAL_MS);
                telemetry.update();
            }
            if (isStopRequested()) return;

            startTime = System.currentTimeMillis();
            long lastRecordTime = 0;

            // Initialize slides and arm to closed/min positions as in DrivingSingleOp
            robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_CLOSED);
            robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_CLOSED);
            robot.armRight.setPosition(ARM_RIGHT_MIN);
            robot.armLeft.setPosition(ARM_LEFT_MIN);


            while (opModeIsActive() && !isStopRequested()) {
                long now = System.currentTimeMillis();

                updateInput();

                // Drive always works
                drive();

                if (waitingRetraction.get()) {
                    // Only show telemetry during retraction
                    telemetry.addLine("RETRACTING - PLEASE WAIT");
                    telemetry.update();

                    // Still record data during retraction
                    if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
                        recordEnhancedData();
                        lastRecordTime = now;
                    }
                    continue;
                }

                if (!waitingRetraction.get()) {
                    // Lift control (same as recorder, but without the arm auto-lift based on lift position)
                    handleLift();

                    // Claw control (updated from DrivingSingleOp)
                    handleClaw();

                    // Arm control (updated from DrivingSingleOp)
                    handleArm();

                    // Extension control (updated from DrivingSingleOp, now on Circle)
                    if (extensionButton.getShortToggle() && !extended)
                        extend();
                }

                // Retraction control (updated from DrivingSingleOp, now on Square)
                if (retractButton.getShortToggle() && !waitingRetraction.get() && extended)
                    retract();
                else if (retractButton.getShortToggle() && waitingRetraction.get())
                    retractButton.resetToggles();
                else if (extensionButton.getShortToggle() && waitingRetraction.get())
                    extensionButton.resetToggles();

                // Handle intake wheels based on 'extended' state
                handleIntakeWheels();


                drive.update();
                robot.gyroscope.updateOrientation();

                // Record data at specified interval
                if (now - lastRecordTime >= RECORD_INTERVAL_MS) {
                    recordEnhancedData();
                    lastRecordTime = now;
                }

                // Telemetry
                telemetry.addData("RECORDING", "Active");
                telemetry.addData("Mode", extended ? "EXTENDED" : "RETRACTED");
                telemetry.addData("Drive Mode", driveModeButton.getLongToggle() ? "Field Centric" : "Robot Centric");
                telemetry.addData("Reverse", reverseButton.getShortToggle() ? "ON" : "OFF");
                telemetry.addData("Lift Position", robot.liftLeft.getCurrentPosition());
                telemetry.update();

                Thread.sleep(10);
            }
        } catch (IOException e) {
            telemetry.addData("ERROR", e.toString());
            telemetry.update();
            throw new RuntimeException(e);
        } finally {
            if (inputRecorder != null) {
                try {
                    inputRecorder.flush();
                    inputRecorder.close();
                } catch (IOException ignored) {}
            }
        }
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
        // Drive controls
        driveModeButton.updateButton(controlGamepad.triangle);
        driveModeButton.longPress();
        reverseButton.updateButton(controlGamepad.cross);
        reverseButton.shortPress();

        // Mechanism controls
        clawButton.updateButton(controlGamepad.dpad_right);
        clawButton.shortPress();

        extensionButton.updateButton(controlGamepad.circle);
        extensionButton.shortPress();

        retractButton.updateButton(controlGamepad.square);
        retractButton.shortPress();

        armButton.updateButton(controlGamepad.dpad_up);
        armButton.shortPress();
    }

    private void drive() {
        robotCentricDrive.setReverse(reverseButton.getShortToggle());
        if (!driveModeButton.getLongToggle()) {
            robotCentricDrive.run();
        } else {
            fieldCentricDrive.run();
        }
    }

    private void extend() {
        extended = true;
        robot.claw.setPosition(CLAW_OPEN);

        robot.intakeServoRight.setPosition(INTAKE_RIGHT_UP);
        robot.intakeServoLeft.setPosition(INTAKE_LEFT_UP);

        robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_OPENED);
        robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_OPENED);

        robot.intakeServoRight.setPosition(INTAKE_RIGHT_MAX);
        robot.intakeServoLeft.setPosition(INTAKE_LEFT_MAX);
    }

    private void retract() {
        extended = false;

        new Thread(() -> {
            waitingRetraction.set(true);

            retractButton.resetToggles();
            extensionButton.resetToggles();

            armButton.resetToggles();
            robot.armRight.setPosition(ARM_RIGHT_MIN);
            robot.armLeft.setPosition(ARM_LEFT_MIN);

            robot.intakeServoRight.setPosition(INTAKE_RIGHT_UP);
            robot.intakeServoLeft.setPosition(INTAKE_LEFT_UP);

            robot.intakeSlideServoRight.setPosition(SLIDE_RIGHT_CLOSED);
            robot.intakeSlideServoLeft.setPosition(SLIDE_LEFT_CLOSED);

            try {
                clawButton.resetToggles();
                robot.claw.setPosition(CLAW_OPEN);
                Thread.sleep(100);

                robot.intakeServoRight.setPosition(INTAKE_RIGHT_MIN);
                robot.intakeServoLeft.setPosition(INTAKE_LEFT_MIN);

                Thread.sleep(300);

                robot.claw.setPosition(CLAW_CLOSE);

                Thread.sleep(200);

                waitingRetraction.set(false);
            } catch (InterruptedException e) {
                e.printStackTrace();
                waitingRetraction.set(false);
            }
        }).start();
    }

    private void handleLift() {
        robot.liftLeft.run(controlGamepad.right_trigger - controlGamepad.left_trigger);
        robot.liftRight.run(controlGamepad.right_trigger - controlGamepad.left_trigger);
    }

    private void handleArm() {
        if (armButton.getShortToggle()) {
            robot.armRight.setPosition(ARM_RIGHT_MAX);
            robot.armLeft.setPosition(ARM_LEFT_MAX);
        } else {
            robot.armRight.setPosition(ARM_RIGHT_MIN);
            robot.armLeft.setPosition(ARM_LEFT_MIN);
        }
    }

    private void handleClaw() {
        if (!extended && !waitingRetraction.get()) {
            if (clawButton.getShortToggle())
                robot.claw.setPosition(CLAW_CLOSE);
            else robot.claw.setPosition(CLAW_OPEN);
        }
    }

    // New method to control intake wheels automatically
    private void handleIntakeWheels() {
        if (extended) {
            // Run intake wheels when extended for collecting (pulling IN)
            // Adjust true/false based on your robot's wiring and desired rotation for INTAKE
            robot.intakeWheelsRight.runContinuous(true, false); // Example: true for forward, false for reverse on this wheel
            robot.intakeWheelsLeft.runContinuous(false, true);  // Example: false for forward, true for reverse on this wheel
        } else {
            // Turn off intake wheels when not extended
            robot.intakeWheelsRight.runContinuous(false, false);
            robot.intakeWheelsLeft.runContinuous(false, false);
        }
    }

    private synchronized void recordEnhancedData() throws IOException {
        double t = (System.currentTimeMillis() - startTime) / 1000.0;
        Pose2d pose = drive.getPoseEstimate();
        double heading = Math.toRadians(robot.gyroscope.getHeading());
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        int liftLeft = robot.liftLeft.getCurrentPosition();
        int liftRight = robot.liftRight.getCurrentPosition();
        double slideLeft = robot.intakeSlideServoLeft.getPosition();
        double slideRight = robot.intakeSlideServoRight.getPosition();
        double clawPos = robot.claw.getPosition();
        double armLeftPos = robot.armLeft.getPosition();
        double armRightPos = robot.armRight.getPosition();
        double intakeLeft = robot.intakeServoLeft.getPosition();
        double intakeRight = robot.intakeServoRight.getPosition();

        // Estimate intake wheel power based on current state (extended)
        // This will now reflect the automatic behavior
        // Assuming 1.0 for positive power (forward) and -1.0 for negative power (reverse)
        double intakeWheelsLeftPower = (extended) ? -1.0 : 0.0; // Changed to negative for intake
        double intakeWheelsRightPower = (extended) ? 1.0 : 0.0; // Changed to positive for intake


        inputRecorder.write(String.format(
                "%.3f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f," +
                        "%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f\n",

                robot.motors.leftRear.getPower(),
                robot.motors.rightRear.getPower(),
                robot.motors.leftFront.getPower(),
                robot.motors.rightFront.getPower(),
                pose.getX(), pose.getY(), heading, voltage,
                liftLeft, liftRight,
                slideLeft, slideRight, clawPos,
                armLeftPos, armRightPos,
                intakeLeft, intakeRight,
                intakeWheelsLeftPower, intakeWheelsRightPower
        ));
        inputRecorder.flush();
    }
}