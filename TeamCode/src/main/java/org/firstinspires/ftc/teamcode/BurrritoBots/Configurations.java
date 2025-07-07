package org.firstinspires.ftc.teamcode.BurrritoBots;
import org.firstinspires.ftc.teamcode.Common.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class Configurations {

    private RevHubOrientationOnRobot.LogoFacingDirection _hubLogoFacingDir = null;
    private RevHubOrientationOnRobot.UsbFacingDirection _hubUsbFacingDir = null;
    private double _maxDriveSpeed;

    public Configurations() {

        //*********************************************************************
        // Note: enter all of your robot configuration parameters here.
        //*********************************************************************

        _hubLogoFacingDir = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        _hubUsbFacingDir = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        _maxDriveSpeed = 0.5;   // max drive speed should NOT exceed 1.0
    }


    public RevHubOrientationOnRobot getHubOrientation() {
        return new RevHubOrientationOnRobot(_hubLogoFacingDir, _hubUsbFacingDir);
    }

    public double getMaxDriveSpeed() {
        return _maxDriveSpeed;
    }

}
