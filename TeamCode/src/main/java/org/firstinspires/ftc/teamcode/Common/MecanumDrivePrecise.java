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

//import static androidx.core.math.MathUtils.clamp;
//import static java.lang.Math.abs;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivePrecise {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ParameterLogger _logger;
    private ImuControlHub _imu;

    // Declare motors
    private DcMotor _frontLeft = null;
    private DcMotor _frontRight = null;
    private DcMotor _backLeft = null;
    private DcMotor _backRight = null;

    // Constants for encoder calculations
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // For GoBILDA 312 RPM Yellow Jacket
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.09449;  // goBuilda 104 mm wheels
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // Number of inches to move when joystick is max (1.0) in any direction.
    private double _inchesPerUnitPower = 12.0;
    private double _totalScaleFactor = COUNTS_PER_INCH * _inchesPerUnitPower;

    private String _paramFLEncVal = "FL Encoder";
    private String _paramFREncVal = "FR Encoder";
    private String _paramBLEncVal = "BL Encoder";
    private String _paramBREncVal = "BR Encoder";

    public MecanumDrivePrecise(boolean shouldResetYaw, RevHubOrientationOnRobot hubOrientation,
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

        // Don't need to waste processing time if we're not moving.
        if ((forward == 0.0) && (right == 0.0)  && (rotate == 0.0) ) {
            setPower(0.0);
            _logger.updateParameter(_paramFLEncVal, _frontLeft.getCurrentPosition());
            _logger.updateParameter(_paramFREncVal, _frontRight.getCurrentPosition());
            _logger.updateParameter(_paramBLEncVal, _backLeft.getCurrentPosition());
            _logger.updateParameter(_paramBREncVal, _backRight.getCurrentPosition());
            _logger.updateAll();
            return;
        }

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

//        double maxPower = 1.0;

//        // This is needed to make sure we don't pass > 1.0 to any wheel
//        // It allows us to keep all of the motors in proportion to what they should
//        // be and not get clipped
//        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));


//        _frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
//        _frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
//        _backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
//        _backRight.setPower(maxSpeed * (backRightPower / maxPower));

        // Calculate new target positions
        int newFLTarget = _frontLeft.getCurrentPosition() + (int)(frontLeftPower * _totalScaleFactor);
        int newFRTarget = _frontRight.getCurrentPosition() + (int)(frontRightPower * _totalScaleFactor);
        int newBLTarget = _backLeft.getCurrentPosition() + (int)(backLeftPower * _totalScaleFactor);
        int newBRTarget = _backRight.getCurrentPosition() + (int)(backRightPower * _totalScaleFactor);

        // Set target positions
        _frontLeft.setTargetPosition(newFLTarget);
        _frontRight.setTargetPosition(newFRTarget);
        _backLeft.setTargetPosition(newBLTarget);
        _backRight.setTargetPosition(newBRTarget);

        // Apply final power
        setPower(maxSpeed);

        _logger.updateParameter(_paramFLEncVal, newFLTarget);
        _logger.updateParameter(_paramFREncVal, newFRTarget);
        _logger.updateParameter(_paramBLEncVal, newBLTarget);
        _logger.updateParameter(_paramBREncVal, newBRTarget);
        _logger.updateAll();
    }

    private void setPower(double power) {
        _frontLeft.setPower(power);
        _frontRight.setPower(power);
        _backLeft.setPower(power);
        _backRight.setPower(power);
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


        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramFLEncVal);
        _logger.addParameter(_paramFREncVal);
        _logger.addParameter(_paramBLEncVal);
        _logger.addParameter(_paramBREncVal);

        resetEncoders();
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

    public void resetEncoders() {
        // Set zero power behavior
        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Switch to RUN_TO_POSITION mode
        _frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set target positions
        // Do this so code doesn't throw error when the very next command is not setTargetPosition()
        _frontLeft.setTargetPosition(0);
        _frontRight.setTargetPosition(0);
        _backLeft.setTargetPosition(0);
        _backRight.setTargetPosition(0);
    }

}
