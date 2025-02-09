package org.firstinspires.ftc.teamcode.kronbot.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;

@TeleOp(name = "Test Servo", group = Constants.TEST_GROUP)
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo1 = hardwareMap.get(Servo.class, "claw");
//        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
//        Servo servo3 = hardwareMap.get(Servo.class, "servo3");
//        Servo servo4 = hardwareMap.get(Servo.class, "servo4");
//        Servo servo5 = hardwareMap.get(Servo.class, "servo5");
//        Servo servo6 = hardwareMap.get(Servo.class, "servo6");
//        Servo servo7 = hardwareMap.get(Servo.class, "servo7");
//        Servo servo8 = hardwareMap.get(Servo.class, "servo8");
//        Servo servo9 = hardwareMap.get(Servo.class, "servo9");
//        Servo servo10 = hardwareMap.get(Servo.class, "servo10");
//        Servo servo11 = hardwareMap.get(Servo.class, "servo11");
//        Servo servo12 = hardwareMap.get(Servo.class, "servo12");

        while (!isStopRequested() && !opModeIsActive()) {}

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            servo1.setPosition(0);
//            servo2.setPosition(0);
//            servo3.setPosition(0);
//            servo4.setPosition(0);
//            servo5.setPosition(0);
//            servo6.setPosition(0);
//            servo7.setPosition(0);
//            servo8.setPosition(0);
//            servo9.setPosition(0);
//            servo10.setPosition(0);
//            servo11.setPosition(0);
//            servo12.setPosition(0);

            sleep(3000);

            servo1.setPosition(1);
//            servo2.setPosition(1);
//            servo3.setPosition(1);
//            servo4.setPosition(1);
//            servo5.setPosition(1);
//            servo6.setPosition(1);
//            servo7.setPosition(1);
//            servo8.setPosition(1);
//            servo9.setPosition(1);
//            servo10.setPosition(1);
//            servo11.setPosition(1);
//            servo12.setPosition(1);


            sleep(3000);
            telemetry.update();
        }
    }
}
