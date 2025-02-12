package org.firstinspires.ftc.teamcode.kronbot.manual;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.MILLIAMPS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.kronbot.KronBot;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Button;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "Imitation Learning", group = Constants.MAIN_GROUP)
public class ImitationLearningOp extends LinearOpMode {
    private final KronBot robot = new KronBot();
    private FileWriter inputRecorder;

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;

    Gamepad drivingGamepad;

    private void recordInput() throws IOException {
        double currentTime = getRuntime();

        double leftRearPower = robot.motors.leftRear.getPower();
        double rightRearPower = robot.motors.rightRear.getPower();
        double leftFrontPower = robot.motors.leftFront.getPower();
        double rightFrontPower = robot.motors.rightFront.getPower();

        double leftRearCurrent = robot.motors.leftRear.getCurrent(MILLIAMPS);
        double rightRearCurrent = robot.motors.rightRear.getCurrent(MILLIAMPS);
        double leftFrontCurrent = robot.motors.leftFront.getCurrent(MILLIAMPS);
        double rightFrontCurrent = robot.motors.rightFront.getCurrent(MILLIAMPS);

        double parallelOdo = -robot.motors.leftFront.getCurrentPosition();
        double perpendicularOdo = robot.motors.rightFront.getCurrentPosition();

        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        double heading = robot.gyroscope.getHeading();

        inputRecorder.write(String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                currentTime, leftRearPower, rightRearPower, leftFrontPower, rightFrontPower,
                leftRearCurrent, rightRearCurrent, leftFrontCurrent, rightFrontCurrent,
                parallelOdo, perpendicularOdo, voltage, heading));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            robot.initSimpleDriving(hardwareMap);

            drivingGamepad = gamepad1;

            robotCentricDrive = new RobotCentricDrive(robot, drivingGamepad);
            fieldCentricDrive = new FieldCentricDrive(robot, drivingGamepad);

            String filePath = Environment.getExternalStorageDirectory().getPath() + "/robot_inputs.csv";
            inputRecorder = new FileWriter(filePath);
            inputRecorder.write("Time,LeftRear Power,RightRear Power,LeftFront Power,RightFront Power," +
                    "LeftRear Current,RightRear Current,LeftFront Current,RightFront Current," +
                    "Parallel Deadwheel Encoder,Perpendicular Deadwheel Encoder,Voltage,Heading\n");

            Button driveModeButton = new Button();
            Button reverseButton = new Button();

            while (!isStopRequested() && !opModeIsActive()) {
                telemetry.addLine("Initialization Ready");
                telemetry.addData("File Path", filePath);
                telemetry.update();
            }

            if (isStopRequested()) return;

            while (opModeIsActive() && !isStopRequested()) {
                driveModeButton.updateButton(drivingGamepad.square);
                driveModeButton.longPress();

                reverseButton.updateButton(drivingGamepad.circle);
                reverseButton.shortPress();

                robotCentricDrive.setReverse(reverseButton.getShortToggle());
                if (!driveModeButton.getLongToggle())
                    robotCentricDrive.run();
                else fieldCentricDrive.run();

                robot.gyroscope.updateOrientation();

                recordInput();

                telemetry.addData("Time", getRuntime());
                telemetry.addData("LeftRear Power", robot.motors.leftRear.getPower());
                telemetry.addData("RightRear Power", robot.motors.rightRear.getPower());
                telemetry.addData("LeftFront Power", robot.motors.leftFront.getPower());
                telemetry.addData("RightFront Power", robot.motors.rightFront.getPower());

                telemetry.addData("leftRear Curent", robot.motors.leftRear.getCurrent(MILLIAMPS));
                telemetry.addData("rightRear Curent", robot.motors.rightRear.getCurrent(MILLIAMPS));
                telemetry.addData("leftFront Curent", robot.motors.leftFront.getCurrent(MILLIAMPS));
                telemetry.addData("rightFront Curent", robot.motors.rightFront.getCurrent(MILLIAMPS));

                telemetry.addData("Parallel Deadwheel Encoder", -robot.motors.leftFront.getCurrentPosition());
                telemetry.addData("Perpendicular Deadwheel Encoder", robot.motors.rightFront.getCurrentPosition());

                telemetry.addData("Voltage", hardwareMap.voltageSensor.iterator().next().getVoltage());

                telemetry.addData("Heading", robot.gyroscope.getHeading());

                telemetry.update();
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        } finally {
            try {
                if (inputRecorder != null) inputRecorder.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
