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

    public static double ROBOT_SPEED = 1.0;
    public static double SLIDES_SPEED = 1.0;
    public static double REST_POWER = 0.05;

    public static double INTAKE_POWER = 1.0;
    public static double HOOK_POWER = 1.0;

    public static double LIFT_TOLERANCE = 50;
    public static int   LIFT_INIT_POSITION = 100 ;
    public static int LIFT_MAX_POSITION = 5000;

    public static double LIFT_REVERSE_CONSTANT = 0.75;

    public static double CAMERA_TRASH_HOLD = 0.2;

    public static int BUTTON_LONG_PRESS_TIME = 750;

    public static int BLUE_HUE_LOW = 0;
    public static int BLUE_HUE_HIGH = 180;

    public static LogoFacingDirection LOGO_FACING_DIRECTION = LogoFacingDirection.UP;
    public static UsbFacingDirection USB_FACING_DIRECTION = UsbFacingDirection.FORWARD;
}
