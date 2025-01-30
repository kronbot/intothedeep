package org.firstinspires.ftc.teamcode.kronbot;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_INT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_INT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_MAX_LEFT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_MAX_RIGHT;

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

    public Servo intakeWheels;
    public com.qualcomm.robotcore.hardware.Servo intakeServoLeft;
    public com.qualcomm.robotcore.hardware.Servo intakeServoRight;
    public com.qualcomm.robotcore.hardware.Servo intakeSlideServoLeft;
    public com.qualcomm.robotcore.hardware.Servo intakeSlideServoRight;

    public com.qualcomm.robotcore.hardware.DcMotor liftMotorLeft;
    public com.qualcomm.robotcore.hardware.DcMotor liftMotorRight;

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
        Motor liftMotorLeft = new Motor(hardwareMap);
        liftLeft = new LiftDriver();
        liftLeft.init(liftMotorLeft, false);
        Motor liftMotorRight = new Motor(hardwareMap);
        liftRight = new LiftDriver();
        liftRight.init(liftMotorRight, true);

//        liftMotorLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "liftMotorLeft");
//        liftMotorLeft.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
////        liftMotorLeft.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
//        liftMotorRight = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "liftMotorRight");
//        liftMotorRight.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initServo(HardwareMap hardwareMap) {
        claw = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "clawServo");

        armLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "armLeftServo");
        armRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "armRightServo");
        armLeft.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        armLeft.setPosition(ARM_LEFT_MAX);
        armRight.setPosition(ARM_RIGHT_MAX);

        intakeWheels = new Servo(hardwareMap);
        intakeWheels.init("intake", true, false, 0, 0, 0);

        intakeServoLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeSlideLeft");
        intakeServoRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeSlideRight");

        intakeSlideServoLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeLeft");
        intakeSlideServoLeft.setPosition(SLIDE_MAX_LEFT);
        intakeSlideServoRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeRight");
        intakeSlideServoRight.setPosition(SLIDE_MAX_RIGHT);
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
