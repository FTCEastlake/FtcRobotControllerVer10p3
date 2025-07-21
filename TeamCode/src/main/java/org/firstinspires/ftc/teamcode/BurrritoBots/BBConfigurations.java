package org.firstinspires.ftc.teamcode.BurrritoBots;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class BBConfigurations {

    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    public static double MAX_DRIVE_SPEED = 0.5; // max drive speed should NOT exceed 1.0

    public BBConfigurations() {
        // Nothing to do here
    }


    public RevHubOrientationOnRobot getHubOrientation() {
        return new RevHubOrientationOnRobot(HUB_LOGO_FACING_DIR, HUB_USB_FACING_DIR);
    }

}
