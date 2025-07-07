package org.firstinspires.ftc.teamcode.BurrritoBots;

import org.firstinspires.ftc.teamcode.Common.ParameterLogger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.ErcCommon.Gobilda4Bar;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BasicTest")
public class BBotBasicTest extends LinearOpMode {

    //**************************************************************
    // Control Hub:
    //    USB port: "webcam 1"
    //    Motor port0: "frontLeft"  (GoBILDA 5202/3/4 series)
    //    Motor port1: "frontRight" (GoBILDA 5202/3/4 series)
    //    Motor port2: "backLeft"   (GoBILDA 5202/3/4 series)
    //    Motor port3: "backRight"  (GoBILDA 5202/3/4 series)
    //    Servo port0: "blinkin led"
    //    I2C port0: "imu"          (REV internal IMU (BHI260AP))
    //    I2C port1: "odo4bar"      (GoBILDA 4-Bar odometry)

    // Declare motors
    private DcMotor _frontLeft = null;
    private DcMotor _frontRight = null;
    private DcMotor _backLeft = null;
    private DcMotor _backRight = null;

    private ParameterLogger _logger;
    private HardwareMap _hardwareMap;

    private ElapsedTime _debounce = new ElapsedTime();

    private String _paramFLEncVal = "FL Encoder";
    private String _paramFREncVal = "FR Encoder";
    private String _paramBLEncVal = "BL Encoder";
    private String _paramBREncVal = "BR Encoder";


    private void initRobot() throws InterruptedException {

        _logger = new ParameterLogger(this, true);
        _hardwareMap = hardwareMap;


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

        //resetEncoders();

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramFLEncVal);
        _logger.addParameter(_paramFREncVal);
        _logger.addParameter(_paramBLEncVal);
        _logger.addParameter(_paramBREncVal);
    }

//    private void resetEncoders() {
//        // Set zero power behavior
//        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Switch to RUN_TO_POSITION mode
//        _frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        _frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        _backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        _backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // Set target positions
//        // Do this so code doesn't throw error when the very next command is not setTargetPosition()
//        _frontLeft.setTargetPosition(0);
//        _frontRight.setTargetPosition(0);
//        _backLeft.setTargetPosition(0);
//        _backRight.setTargetPosition(0);
//    }

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        while (!isStarted())
        {
            // Add code to run before the start button is pressed
            _logger.updateStatus("left_stick_y = " + gamepad1.left_stick_y);
            _logger.updateAll();

        }

        double drivePower, loopCount = 0.0;

        //******************************
        // Main loop
        //******************************
        waitForStart();
        _logger.resetCycleTimer();
        while (!isStopRequested())
        {
            // Note: push start and A button on the gamepad to enable gamepad1
            double power = gamepad1.left_stick_y;
            int encoderVal = (int)(power * 30);
            DcMotor targetWheel = null;
            if (gamepad1.x) targetWheel = _frontLeft;       //_frontLeft.setPower(power);
            else if (gamepad1.y) targetWheel = _frontRight; //_frontRight.setPower(power);
            else if (gamepad1.a) targetWheel = _backLeft;   //_backLeft.setPower(power);
            else if (gamepad1.b) targetWheel = _backRight;  //_backRight.setPower(power);
            if (targetWheel != null)
            {
                targetWheel.setTargetPosition(targetWheel.getCurrentPosition() + encoderVal);
                targetWheel.setPower(power);
            }


            if (gamepad1.right_bumper)
            {
                _frontLeft.setTargetPosition(_frontLeft.getCurrentPosition() + encoderVal);
                _frontRight.setTargetPosition(_frontRight.getCurrentPosition() + encoderVal);
                _backLeft.setTargetPosition(_backLeft.getCurrentPosition() + encoderVal);
                _backRight.setTargetPosition(_backRight.getCurrentPosition() + encoderVal);
                _frontLeft.setPower(power);
                _frontRight.setPower(power);
                _backLeft.setPower(power);
                _backRight.setPower(power);

            }

            _logger.updateStatus("left_stick_y = " + gamepad1.left_stick_y);
            _logger.updateParameter(_paramFLEncVal, _frontLeft.getCurrentPosition());
            _logger.updateParameter(_paramFREncVal, _frontRight.getCurrentPosition());
            _logger.updateParameter(_paramBLEncVal, _backLeft.getCurrentPosition());
            _logger.updateParameter(_paramBREncVal, _backRight.getCurrentPosition());
            _logger.updateAll();

        }
    }








}
