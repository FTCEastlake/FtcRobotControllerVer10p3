package org.firstinspires.ftc.teamcode.BurrritoBots;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class BBConfigurations {

    public BBConfigurations() {
        // Nothing to do here
    }

    // Drivetrain
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    public static double MAX_DRIVE_SPEED = 0.5; // max drive speed should NOT exceed 1.0
    public static RevHubOrientationOnRobot getHubOrientation() {
        return new RevHubOrientationOnRobot(HUB_LOGO_FACING_DIR, HUB_USB_FACING_DIR);
    }

    // Claw
    public static double MAX_CLAW_POSITION = 0.28;

    // Rotation
    public static int MIN_ARM_ROTATION_ENCODER_VAL = 0;
    public static int MAX_ARM_ROTATION_ENCODER_VAL = 0;


    public static double AUTON_DRIVE_SPEED = 0.5;

}
