package org.firstinspires.ftc.teamcode.kronbot.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

/**
 * Constants for KronBot viewable from dashboard
 *
 * @version 1.0
 */
@Config
public class Constants {
    public final static String TEST_GROUP = "test";
    public final static String MAIN_GROUP = "main";

    public static double CONTROLLER_DEADZONE = 0.15;

    public static int BUTTON_LONG_PRESS_TIME = 750;

    public static double ROBOT_SPEED = 1.0;
    public static double POWER_EXPONENT = 2.0;

    public static double LIFT_POWER = 1.0;
    public static double LIFT_REST_POWER = 0;
    public static double LIFT_REVERSE_POWER = 0.75;
    public static double LIFT_TOLERANCE = 50;
    public static int   LIFT_INIT_POSITION = 0 ;
    public static int LIFT_MAX_POSITION = 3100;

    public static double CAMERA_TRASH_HOLD = 0.2;
    public static int BLUE_HUE_LOW = 0;
    public static int BLUE_HUE_HIGH = 180;

    public static double ARM_LEFT_MIN = 0.86;
    public static double ARM_LEFT_MAX = 0.25;
    public static double ARM_LEFT_INT = 0.45;

    public static double ARM_RIGHT_MIN = 0.86;
    public static double ARM_RIGHT_MAX = 0.25;
    public static double ARM_RIGHT_INT = 0.45;

    public static double CLAW_CLOSE = 0.45;
    public static double CLAW_OPEN = 0.23;

    public static double INTAKE_LEFT_MIN = 0.20;
    public static double INTAKE_LEFT_MAX = 0.80;
    public static double INTAKE_RIGHT_MIN = 0.20;
    public static double INTAKE_RIGHT_MAX = 0.80;

    public static double SLIDE_LEFT_CLOSED = 0.26;
    public static double SLIDE_LEFT_OPENED = 1;
    public static double SLIDE_RIGHT_CLOSED = 0.1;
    public static double SLIDE_RIGHT_OPENED = 1;

    public static LogoFacingDirection LOGO_FACING_DIRECTION = LogoFacingDirection.LEFT;
    public static UsbFacingDirection USB_FACING_DIRECTION = UsbFacingDirection.UP;
}
