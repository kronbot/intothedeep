package org.firstinspires.ftc.teamcode.kronbot;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_INIT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_INIT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.INTAKE_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_LEFT_INIT;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.SLIDE_RIGHT_INIT;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
//
// import com.qualcomm.robotcore.hardware.Servo;

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

    public com.qualcomm.robotcore.hardware.Servo intakeServoLeft;
    public com.qualcomm.robotcore.hardware.Servo intakeServoRight;
    public com.qualcomm.robotcore.hardware.Servo intakeSlideServoLeft;
    public com.qualcomm.robotcore.hardware.Servo intakeSlideServoRight;
    public com.qualcomm.robotcore.hardware.Servo intakeClawServo;
    public com.qualcomm.robotcore.hardware.Servo intakeLiftServo;
    public com.qualcomm.robotcore.hardware.Servo intakeRotateServo;


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
        liftLeft.init(new Motor(hardwareMap), true, "liftMotorLeft", false);
        liftRight.init(new Motor(hardwareMap), true, "liftMotorRight", true);
    }

    public void initServo(HardwareMap hardwareMap) {

        //testServo = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "test")

        claw = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "clawServo");

        intakeClawServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeServo");
        intakeClawServo.setPosition(CLAW_OPEN);
        intakeLiftServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeServo");
        intakeLiftServo.setPosition(CLAW_OPEN);
        intakeRotateServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeRotateServo");

        armLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "armLeftServo");
        armRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "armRightServo");
        armLeft.setPosition(ARM_LEFT_INIT);
        armRight.setPosition(ARM_RIGHT_INIT);

        intakeServoLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeLeft");
        intakeServoLeft.setPosition(INTAKE_LEFT_MIN);
        intakeServoLeft.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        intakeServoRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeRight");
        intakeServoRight.setPosition(INTAKE_RIGHT_MIN);

        intakeSlideServoLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeSlideLeft");
        intakeSlideServoLeft.setPosition(SLIDE_LEFT_INIT);
        intakeSlideServoLeft.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        intakeSlideServoRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeSlideRight");
        intakeSlideServoRight.setPosition(SLIDE_RIGHT_INIT);
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