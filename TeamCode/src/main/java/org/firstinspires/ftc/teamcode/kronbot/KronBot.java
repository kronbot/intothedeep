package org.firstinspires.ftc.teamcode.kronbot;

import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_LEFT_START;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.ARM_RIGHT_START;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.CLAW_START;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HAND_MAX;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HAND_MIN;
import static org.firstinspires.ftc.teamcode.kronbot.utils.Constants.HAND_START;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.kronbot.utils.Constants;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.LiftDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.drivers.MotorDriver;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.ControlHubGyroscope;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Motor;
import org.firstinspires.ftc.teamcode.kronbot.utils.wrappers.Servo;

public class KronBot {
    public MotorDriver motors;
    public LiftDriver lift;

    public Servo armLeft;
    public Servo armRight;
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
        armLeft = new Servo(hardwareMap);
        armLeft.init("armLeftServo", false, false, ARM_LEFT_MIN, ARM_LEFT_MAX, ARM_LEFT_START);
        armRight = new Servo(hardwareMap);
        armRight.init("armRightServo", false, false, ARM_RIGHT_MIN, ARM_RIGHT_MAX, ARM_RIGHT_START);
        claw = new Servo(hardwareMap);
        claw.init("clawServo", false, false, CLAW_MIN, CLAW_MAX, CLAW_START);
        hand = new Servo(hardwareMap);
        hand.init("handServo", false, false, HAND_MIN, HAND_MAX, HAND_START);
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
