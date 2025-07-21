package org.firstinspires.ftc.teamcode.kronbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "ImprovedAuto", group = "Autonomous")
public class ImprovedAuto extends LinearOpMode {
    private static final String CSV_PATH         = "/sdcard/robot_inputs.csv";
    private static final double MAX_POWER        = 0.8;
    private static final double kP_translation   = 0.08;
    private static final double kD_translation   = 0.02;
    private static final double kP_rotation      = 1.5;
    private static final double kD_rotation      = 0.1;
    private static final double LIFT_KP          = 0.01;
    private static final int    LIFT_TOLERANCE   = 10;

    // Battery compensation constants
    private static final double NOMINAL_VOLTAGE  = 12.0;  // Voltage when recording was done
    private static final double MIN_VOLTAGE      = 10.5;  // Minimum safe operating voltage
    private static final double TIME_SCALING_FACTOR = 0.7; // How much to slow down timeline when voltage is low

    private KronBot robot = new KronBot();
    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();
    private List<RobotFrame> recordedFrames = new ArrayList<>();

    // PD state
    private double prevErrorX = 0, prevErrorY = 0, prevErrorHeading = 0, prevTime = 0;

    // Battery compensation
    private double recordedVoltage = NOMINAL_VOLTAGE;
    private double timeScalingFactor = 1.0;

    private static class RobotFrame {
        double timestamp;
        double leftRearPower, rightRearPower, leftFrontPower, rightFrontPower;
        double xPosition, yPosition, heading, voltage;
        int    liftLeftPos, liftRightPos;
        double slideLeftPos, slideRightPos, clawPos;
        double armLeftPos, armRightPos;
        double intakeLeftPos, intakeRightPos;
        double intakeLeftPower, intakeRightPower;

        RobotFrame(String[] d) {
            timestamp        = Double.parseDouble(d[0]);
            leftRearPower    = Double.parseDouble(d[1]);
            rightRearPower   = Double.parseDouble(d[2]);
            leftFrontPower   = Double.parseDouble(d[3]);
            rightFrontPower  = Double.parseDouble(d[4]);
            xPosition        = Double.parseDouble(d[5]);
            yPosition        = Double.parseDouble(d[6]);
            heading          = Math.toRadians(Double.parseDouble(d[7]));
            voltage          = Double.parseDouble(d[8]);
            liftLeftPos      = Integer.parseInt(d[9]);
            liftRightPos     = Integer.parseInt(d[10]);
            slideLeftPos     = Double.parseDouble(d[11]);
            slideRightPos    = Double.parseDouble(d[12]);
            clawPos          = Double.parseDouble(d[13]);
            armLeftPos       = Double.parseDouble(d[14]);
            armRightPos      = Double.parseDouble(d[15]);
            intakeLeftPos    = Double.parseDouble(d[16]);
            intakeRightPos   = Double.parseDouble(d[17]);
            intakeLeftPower  = Double.parseDouble(d[18]);
            intakeRightPower = Double.parseDouble(d[19]);
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Direct Path Follower...");
        telemetry.update();

        // init everything via KronBot wrapper
        robot.initTeleop(hardwareMap);

        try {
            // init drive
            drive = new SampleMecanumDrive(hardwareMap);
            drive.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // load data
            telemetry.addLine("Loading recorded data...");
            telemetry.update();
            loadRecordedData();
            if (recordedFrames.isEmpty()) {
                telemetry.addLine("ERROR: No recorded data found!");
                telemetry.update();
                return;
            }

            // Calculate average recorded voltage for compensation
            calculateRecordedVoltage();

            // set start pose
            RobotFrame startFrame = recordedFrames.get(0);
            Pose2d startPose = new Pose2d(
                    startFrame.xPosition,
                    startFrame.yPosition,
                    startFrame.heading
            );
            drive.setPoseEstimate(startPose);

            telemetry.addLine("Initialization Complete");
            telemetry.addData("Frames Loaded", recordedFrames.size());
            telemetry.addData("Recorded Voltage", "%.1f V", recordedVoltage);
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            telemetry.addLine("=== AUTONOMOUS STARTED ===");
            telemetry.update();

            executeDirectPlayback();

            drive.setMotorPowers(0,0,0,0);
            telemetry.addLine("=== PATH FOLLOWING COMPLETE ===");
            telemetry.addData("Total Runtime", "%.2f s", runtime.seconds());
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("=== ERROR OCCURRED ===");
            telemetry.addData("Error", e.toString());
            telemetry.update();
            if (drive != null) drive.setMotorPowers(0,0,0,0);
            sleep(5000);
        }
    }

    private void loadRecordedData() throws IOException {
        File csvFile = new File(CSV_PATH);
        if (!csvFile.exists()) throw new IOException("CSV not found: " + CSV_PATH);
        try (BufferedReader reader = new BufferedReader(new FileReader(csvFile))) {
            reader.readLine(); // skip header
            String line;
            while ((line = reader.readLine()) != null) {
                String[] d = line.split(",");
                if (d.length >= 20) recordedFrames.add(new RobotFrame(d));
            }
        }
    }

    private void calculateRecordedVoltage() {
        if (recordedFrames.isEmpty()) return;

        double totalVoltage = 0;
        int validReadings = 0;

        for (RobotFrame frame : recordedFrames) {
            if (frame.voltage > 0) {  // Only count valid voltage readings
                totalVoltage += frame.voltage;
                validReadings++;
            }
        }

        if (validReadings > 0) {
            recordedVoltage = totalVoltage / validReadings;
        }
    }

    private void updateTimeScaling() {
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Calculate how much slower the robot will be due to voltage drop
        double voltageRatio = currentVoltage / recordedVoltage;

        if (voltageRatio < 1.0) {
            // Robot will be slower, so we need to slow down the timeline
            // This is a conservative estimate - you may need to tune this based on testing
            timeScalingFactor = 1.0 + TIME_SCALING_FACTOR * (1.0 - voltageRatio);
        } else {
            // Battery is same or better than when recorded
            timeScalingFactor = 1.0;
        }

        // Reasonable bounds - don't slow down too much or speed up
        timeScalingFactor = Range.clip(timeScalingFactor, 1.0, 2.0);
    }

    private void executeDirectPlayback() {
        runtime.reset();
        double startTs = recordedFrames.get(0).timestamp;
        double endTs   = recordedFrames.get(recordedFrames.size()-1).timestamp;
        double duration= endTs - startTs;
        int idx = 0;
        prevTime = 0;

        while (opModeIsActive() && idx < recordedFrames.size()-1) {
            double now      = runtime.seconds();

            // Update time scaling based on current voltage
            updateTimeScaling();

            // Slow down the timeline if voltage is low - this is the key insight!
            // Instead of trying to make motors faster, we give them more time
            double adjustedTime = now / timeScalingFactor;
            double targetTs = startTs + adjustedTime;

            while (idx < recordedFrames.size()-1
                    && recordedFrames.get(idx+1).timestamp <= targetTs) {
                idx++;
            }

            RobotFrame tf = recordedFrames.get(idx);
            Pose2d targetPose = new Pose2d(tf.xPosition, tf.yPosition, tf.heading);

            drive.updatePoseEstimate();
            Pose2d curPose = drive.getPoseEstimate();

            Vector2d dv  = calculateDriveVector(curPose, targetPose, now);
            double rotPw = calculateRotationalPower(curPose, targetPose, now);
            drive.setWeightedDrivePower(new Pose2d(dv.getX(), dv.getY(), rotPw));

            controlMechanisms(tf);

            double posErr  = Math.hypot(
                    targetPose.getX()-curPose.getX(),
                    targetPose.getY()-curPose.getY()
            );
            double headErr = Math.toDegrees(
                    Math.abs(normalizeAngle(targetPose.getHeading() - curPose.getHeading()))
            );

            double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            telemetry.addData("Time",  "%.2f/%.2f s (%.1fx)", now, duration, timeScalingFactor);
            telemetry.addData("Frame","%d/%d", idx, recordedFrames.size());
            telemetry.addData("PosErr","%.2f cm", posErr);
            telemetry.addData("HeadErr","%.1f°", headErr);
            telemetry.addData("Voltage", "%.1f V (rec: %.1f V)", currentVoltage, recordedVoltage);
            telemetry.update();

            sleep(20);
        }
    }

    private Vector2d calculateDriveVector(Pose2d cur, Pose2d tgt, double now) {
        double ex = tgt.getX() - cur.getX();
        double ey = tgt.getY() - cur.getY();
        double dt = now - prevTime;
        double dx = dt>0 ? (ex - prevErrorX)/dt : 0;
        double dy = dt>0 ? (ey - prevErrorY)/dt : 0;

        double fx = ex*kP_translation + dx*kD_translation;
        double fy = ey*kP_translation + dy*kD_translation;

        double ca = cur.getHeading(), sa = Math.sin(ca), co = Math.cos(ca);
        Vector2d field = new Vector2d(fx, fy);
        Vector2d robotVec = new Vector2d(
                co*field.getX() + sa*field.getY(),
                -sa*field.getX()+ co*field.getY()
        );
        if (robotVec.norm()>MAX_POWER) {
            robotVec = robotVec.times(MAX_POWER/robotVec.norm());
        }

        prevErrorX = ex;
        prevErrorY = ey;
        prevTime   = now;
        return robotVec;
    }

    private double calculateRotationalPower(Pose2d cur, Pose2d tgt, double now) {
        // FIXED: Use normalizeAngle to handle wrap-around properly
        double eh = normalizeAngle(tgt.getHeading() - cur.getHeading());
        double dt = now - prevTime;
        double dh = dt>0 ? (eh - prevErrorHeading)/dt : 0;
        double p  = eh*kP_rotation + dh*kD_rotation;
        prevErrorHeading = eh;
        return Range.clip(p, -MAX_POWER, MAX_POWER);
    }

    private void controlMechanisms(RobotFrame t) {
        // lift - keep original powers since we're compensating with time instead
        int cl = robot.liftLeft.getCurrentPosition();
        int cr = robot.liftRight.getCurrentPosition();

        robot.liftLeft.run(Math.abs(t.liftLeftPos-cl)>LIFT_TOLERANCE
                ? Range.clip((t.liftLeftPos-cl)*LIFT_KP, -1,1) : 0);
        robot.liftRight.run(Math.abs(t.liftRightPos-cr)>LIFT_TOLERANCE
                ? Range.clip((t.liftRightPos-cr)*LIFT_KP, -1,1) : 0);

        // slides, claw, arm, intake servos
        robot.intakeSlideServoLeft .setPosition(t.slideLeftPos);
        robot.intakeSlideServoRight.setPosition(t.slideRightPos);
        robot.claw                  .setPosition(t.clawPos);
        robot.armLeft               .setPosition(t.armLeftPos);
        robot.armRight              .setPosition(t.armRightPos);
        robot.intakeServoLeft       .setPosition(t.intakeLeftPos);
        robot.intakeServoRight      .setPosition(t.intakeRightPos);

        // intake wheels - keep original powers
        robot.intakeWheelsLeft .runContinuous(t.intakeLeftPower>0,  t.intakeLeftPower<0);
        robot.intakeWheelsRight.runContinuous(t.intakeRightPower>0, t.intakeRightPower<0);
    }

    /**
     * Normalizes an angle to the range [-π, π]
     * This fixes the heading wrap-around issue
     */
    private static double normalizeAngle(double a) {
        while (a > Math.PI)  a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}