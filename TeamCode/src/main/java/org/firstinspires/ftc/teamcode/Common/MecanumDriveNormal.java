package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveNormal {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ParameterLogger _logger;
    private ImuControlHub _imu;

    // Declare motors
    private DcMotor _frontLeft = null;
    private DcMotor _frontRight = null;
    private DcMotor _backLeft = null;
    private DcMotor _backRight = null;


    public MecanumDriveNormal(boolean shouldResetYaw, RevHubOrientationOnRobot hubOrientation,
                              LinearOpMode opMode, ParameterLogger logger) throws InterruptedException{
        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;
        _imu = new ImuControlHub(hubOrientation, _opMode, _logger);

        init(shouldResetYaw);
    }

    // This routine drives the robot field relative
    public void driveFieldRelative(double forward, double right, double rotate, double maxSpeed) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                _imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate, maxSpeed);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate, double maxSpeed) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));


        _frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        _frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        _backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        _backRight.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void init(boolean shouldResetYaw) throws InterruptedException {

        // Make sure your ID's match your configuration
        _frontLeft = _hardwareMap.get(DcMotor.class, "frontLeft");
        _frontRight = _hardwareMap.get(DcMotor.class, "frontRight");
        _backLeft = _hardwareMap.get(DcMotor.class, "backLeft");
        _backRight = _hardwareMap.get(DcMotor.class, "backRight");

        // Set directions of the motors.
        // Left motors should be the opposite of the right motors.
        _frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        _backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        _frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        _backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        if (shouldResetYaw)
            resetYaw();

    }

    public void resetYaw() {
        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        _imu.resetYaw();
        _logger.updateStatus("IMU reset yaw");
        _logger.updateAll();
    }

}
