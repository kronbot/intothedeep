package org.firstinspires.ftc.teamcode.kronbot;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_START;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_START;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HAND_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HAND_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HAND_START;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.LiftDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class KronBot {
    public MotorDriver motors;
    public LiftDriver lift;

    public com.qualcomm.robotcore.hardware.Servo armLeft;
    public com.qualcomm.robotcore.hardware.Servo armRight;
    public Servo claw;
    public Servo hand;

    public ControlHubGyroscope gyroscope;

    public void initMotors(HardwareMap hardwareMap) {
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        motors = new MotorDriver();
        motors.init(leftRear, leftFront, rightRear, rightFront);
    }

    public void initIMU(HardwareMap hardwareMap) {
        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        gyroscope = new ControlHubGyroscope(hardwareMap);
        gyroscope.init(imu);
    }

    public void initLift(HardwareMap hardwareMap) {
        Motor liftMotor = new Motor(hardwareMap);
        lift = new LiftDriver();
        lift.init(liftMotor, false);
    }

    public void initServo(HardwareMap hardwareMap) {
        claw = new Servo(hardwareMap);
        claw.init("clawServo", false, false, CLAW_CLOSE, CLAW_OPEN, CLAW_OPEN);
        hand = new Servo(hardwareMap);
        hand.init("handServo", false, false, HAND_MIN, HAND_MAX, HAND_START);

        armLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "armLeftServo");
        armLeft.setPosition(ARM_LEFT_START);
        armRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "armRightServo");
        armRight.setPosition(ARM_RIGHT_START);

    }

    public void initAutonomy(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initServo(hardwareMap);
        initLift(hardwareMap);
        initIMU(hardwareMap);
    }

    public void initTeleop(HardwareMap hardwareMap) {
        initMotors(hardwareMap);
        initLift(hardwareMap);
        initServo(hardwareMap);
        initIMU(hardwareMap);
    }

    public void initSimpleDriving(HardwareMap hardwareMap) {
        initIMU(hardwareMap);
        initMotors(hardwareMap);
    }

}
