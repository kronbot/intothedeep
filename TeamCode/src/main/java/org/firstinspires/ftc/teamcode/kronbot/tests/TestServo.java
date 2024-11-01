package org.firstinspires.ftc.teamcode.kronbot.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

@TeleOp(name = "Test Servo", group = Constants.TEST_GROUP)
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoTest;
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        while (!isStopRequested() && !opModeIsActive()) {}

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            servoTest.setPosition(0.5);

            telemetry.update();
        }
    }
}
