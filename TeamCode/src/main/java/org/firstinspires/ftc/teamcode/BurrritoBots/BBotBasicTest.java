package org.firstinspires.ftc.teamcode.BurrritoBots;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.ParameterLogger;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

// FTC dashboard
import com.acmerobotics.dashboard.FtcDashboard;

//import org.firstinspires.ftc.teamcode.ErcCommon.Gobilda4Bar;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BBotBasicTest")
//@Disabled
public class BBotBasicTest extends LinearOpMode {

    //**************************************************************
    // Control Hub:
    //    USB port: "webcam 1"
    //    Motor port0: "backRight"  (GoBILDA 5202/3/4 series)
    //    Motor port1: "backLeft"   (GoBILDA 5202/3/4 series)
    //    Motor port2: "frontRight" (GoBILDA 5202/3/4 series)
    //    Motor port3: "frontLeft"  (GoBILDA 5202/3/4 series)
    //    Servo port0: "blinkin led"
    //    I2C port0: "imu"          (REV internal IMU (BHI260AP))
    //    I2C port1: "odo4bar"      (GoBILDA Pinpoint Odometry Computer)
    //
    // Expansion Hub:
    //    USB port: "webcam 1"
    //    Motor port0: "rotationMotor"  (GoBILDA 5202/3/4 series)
    //    Motor port1: "slideMotor"     (GoBILDA 5202/3/4 series)
    //    Motor port2: Nothing
    //    Motor port3: Nothing
    //    Servo port0: "clawServo"      (Servo)
    //    I2C port0: "imu2"             (REV internal IMU (BNO055))
    //    I2C port1: None
    //
    // WebcamFront

    // Declare motors
    private DcMotor _frontLeft = null;
    private DcMotor _frontRight = null;
    private DcMotor _backLeft = null;
    private DcMotor _backRight = null;

    private ParameterLogger _logger;
    private HardwareMap _hardwareMap;
    private BBVision _vision = null;

    //private BBConfigurations _configs;
    private BBArmClaw _armClaw;
    private BBArmRotate _armRotate;
    private BBArmSlide _armSlide;

    private double _lsy;
    private double _rsy;

    private ElapsedTime _debounce = new ElapsedTime();


    private String _paramFLEncVal = "FL Encoder";
    private String _paramFREncVal = "FR Encoder";
    private String _paramBLEncVal = "BL Encoder";
    private String _paramBREncVal = "BR Encoder";


    private void initRobot() throws InterruptedException {

        _logger = new ParameterLogger(this, true);
        _hardwareMap = hardwareMap;
        //_configs = new BBConfigurations();
        _armClaw = new BBArmClaw(this, _logger);
        _armRotate = new BBArmRotate(this, _logger);
        _armSlide = new BBArmSlide(this, _logger);

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

        //_vision = new BBVision(false, false, this, _logger);
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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

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

        // Indexed according to the port number
        String[] motorAliases = {"backRight", "backLeft", "frontRight", "frontLeft"};
        waitForStart();
        _logger.resetCycleTimer();
        DcMotor targetWheel = null;
        while (!isStopRequested())
        {
            // Note: push start and A button on the gamepad to enable gamepad1
            double power = gamepad1.left_stick_y * BBConfigurations.MAX_DRIVE_SPEED;
            int wheelEncoderVal = (int)(power * 30);
            if (gamepad1.x) targetWheel = _frontLeft;       //_frontLeft.setPower(power);
            else if (gamepad1.y) targetWheel = _frontRight; //_frontRight.setPower(power);
            else if (gamepad1.a) targetWheel = _backLeft;   //_backLeft.setPower(power);
            else if (gamepad1.b) targetWheel = _backRight;  //_backRight.setPower(power);
            if (targetWheel != null)
            {
                targetWheel.setTargetPosition(targetWheel.getCurrentPosition() + wheelEncoderVal);
                targetWheel.setPower(power);

                // Unfortunately targetWheel.getDeviceName() returns "DcMotor" string and not the alias (ex: "frontLeft")
                // You can use the following loop to get the alias
//                for (Map.Entry<String, DcMotor> entry : _hardwareMap.dcMotor.entrySet()) {
//                    if (entry.getValue() == targetWheel) {
//                        dashboardTelemetry.addData("targetWheel", entry.getKey());
//                        break;
//                    }
//                }
                // Or use the more efficient string array index
                dashboardTelemetry.addData("targetWheel", motorAliases[targetWheel.getPortNumber()]);
                dashboardTelemetry.addData("power", power);
                dashboardTelemetry.update();
            }


            if (gamepad1.right_bumper)
            {
                _frontLeft.setTargetPosition(_frontLeft.getCurrentPosition() + wheelEncoderVal);
                _frontRight.setTargetPosition(_frontRight.getCurrentPosition() + wheelEncoderVal);
                _backLeft.setTargetPosition(_backLeft.getCurrentPosition() + wheelEncoderVal);
                _backRight.setTargetPosition(_backRight.getCurrentPosition() + wheelEncoderVal);
                _frontLeft.setPower(power);
                _frontRight.setPower(power);
                _backLeft.setPower(power);
                _backRight.setPower(power);

            }

            if (gamepad2.b) _armClaw.setClawOpen();
            if (gamepad2.a) _armClaw.setClawClose();

            _rsy = -gamepad2.right_stick_y;
            _armRotate.setRotationNormalMode(_rsy);

            _lsy = -gamepad2.left_stick_y;;
            _armSlide.setSlideNormalMode(_lsy);


            _logger.updateStatus("left_stick_y = " + gamepad1.left_stick_y);
            _logger.updateParameter(_paramFLEncVal, _frontLeft.getCurrentPosition());
            _logger.updateParameter(_paramFREncVal, _frontRight.getCurrentPosition());
            _logger.updateParameter(_paramBLEncVal, _backLeft.getCurrentPosition());
            _logger.updateParameter(_paramBREncVal, _backRight.getCurrentPosition());
            _logger.updateAll();

        }
    }


}
