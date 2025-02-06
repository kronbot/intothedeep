package org.firstinspires.ftc.teamcode.kronbot;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_CLOSED;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_CLOSED;

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

    public LiftDriver liftLeft;
    public LiftDriver liftRight;

    public com.qualcomm.robotcore.hardware.Servo armLeft;
    public com.qualcomm.robotcore.hardware.Servo armRight;
    public com.qualcomm.robotcore.hardware.Servo claw;

    public Servo intakeWheelsRight;
    public Servo intakeWheelsLeft;
    public com.qualcomm.robotcore.hardware.Servo intakeServoLeft;
    public com.qualcomm.robotcore.hardware.Servo intakeServoRight;
    public com.qualcomm.robotcore.hardware.Servo intakeSlideServoLeft;
    public com.qualcomm.robotcore.hardware.Servo intakeSlideServoRight;

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
        liftLeft = new LiftDriver();
        liftRight = new LiftDriver();
        liftLeft.init(new Motor(hardwareMap), true, "liftMotorLeft");
        liftRight.init(new Motor(hardwareMap), true, "liftMotorRight");
    }

    public void initServo(HardwareMap hardwareMap) {
        claw = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "clawServo");

        armLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "armLeftServo");
        armRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "armRightServo");
        armLeft.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        armLeft.setPosition(ARM_LEFT_MIN);
        armRight.setPosition(ARM_RIGHT_MIN);

        intakeWheelsRight = new Servo(hardwareMap);
        intakeWheelsRight.init("intake2", true, false, 0, 0, 0);
        intakeWheelsLeft = new Servo(hardwareMap);
        intakeWheelsLeft.init("intake", true, false, 0, 0, 0);
        intakeWheelsLeft.setReversed(true);

        intakeServoLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeLeft");
        intakeServoLeft.setPosition(INTAKE_LEFT_MIN);
        intakeServoRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeRight");
        intakeServoRight.setPosition(INTAKE_RIGHT_MIN);

        intakeSlideServoLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeSlideLeft");
        intakeSlideServoLeft.setPosition(SLIDE_LEFT_CLOSED);
        intakeSlideServoLeft.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        intakeSlideServoRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeSlideRight");
        intakeSlideServoRight.setPosition(SLIDE_RIGHT_CLOSED);
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