package org.firstinspires.ftc.teamcode.BurrritoBots;
import org.firstinspires.ftc.teamcode.Common.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name = "BurrritoBotsAuto")
//@Disabled
@TeleOp(name = "BurrritoBotsAuto")
public class BurrritoBotsAuto extends LinearOpMode {

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

    private ParameterLogger _logger;
    private HardwareMap _hardwareMap;
    private BBVision _vision = null;

    //private BBConfigurations _configs;
    private MecanumDrivePrecise _drive;
    private BBArmClaw _armClaw;
    private BBArmRotate _armRotate;
    private BBArmSlide _armSlide;

    private double _lsy;
    private double _rsy;

    private ElapsedTime _debounce = new ElapsedTime();

    private double _maxDriveSpeed;


    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        while (!isStarted()) {
            // Add code
            _logger.updateStatus("isStarted = false");
            _logger.updateAll();
        }
        waitForStart();
        _logger.resetCycleTimer();

        //******************************
        // Main loop
        //******************************
        boolean isDone = false;
        double forward = 0.0, right = 0.0, rotate = 0.0, drivePower = 0.0;
        double armRotatePower = 0.5, armSlidePower = 0.5;
        int armRotateEncoderVal = 0, armSlideEncoderVal = 0;
        while (!isStopRequested()) {

            if (isDone)
                continue;

            drivePower = BBConfigurations.AUTON_DRIVE_SPEED;

//            forward = 1.0; right = 0.0; rotate = 0.0; _drive.drive(forward, right, rotate, drivePower);
//            forward = -1.0; right = 0.0; rotate = 0.0; _drive.drive(forward, right, rotate, drivePower);
//            forward = 0.0; right = 0.25; rotate = 0.0; _drive.drive(forward, right, rotate, drivePower);
//            forward = 0.0; right = -0.25; rotate = 0.0; _drive.drive(forward, right, rotate, drivePower);
//            forward = 0.0; right = 0.0; rotate = 0.25; _drive.drive(forward, right, rotate, drivePower);
//            forward = 0.0; right = 0.0; rotate = -0.25; _drive.drive(forward, right, rotate, drivePower);

            armRotateEncoderVal = 500; _armRotate.setRotationAutoMode(armRotateEncoderVal, armRotatePower);
            armSlideEncoderVal = 500; _armSlide.setRotationAutoMode(armSlideEncoderVal, armSlidePower);
            _armClaw.setClawOpen();
            armSlideEncoderVal = 0; _armSlide.setRotationAutoMode(armSlideEncoderVal, armSlidePower);
            _armClaw.setClawClose();
            armRotateEncoderVal = 0; _armRotate.setRotationAutoMode(armRotateEncoderVal, armRotatePower);


//
//            _logger.updateStatus("isStopRequested = false");
//            _logger.updateAll();
//
//            //****************************************
//            // Drive
//            //****************************************
//            // If you press the start button, then you reset the Yaw to be zero from the way
//            // the robot is currently pointing
//            if (gamepad1.start) {
//                _drive.resetYaw();
//            }
//
//            // This will allow you to update the max speed on the fly through FTC dashboard
//            _maxDriveSpeed = _configs.MAX_DRIVE_SPEED;
//
//            // If you press the left bumper, you get a drive from the point of view of the robot
//            // (much like driving an RC vehicle)
//            if (gamepad1.left_bumper) {
//                _drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, _maxDriveSpeed);
//            } else {
//                _drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, _maxDriveSpeed);
//            }
//
//            //****************************************
//            // Claw
//            //****************************************
//            if (gamepad2.b) _claw.setClawOpen();
//            if (gamepad2.a) _claw.setClawClose();
//
//            //****************************************
//            // Arm rotate
//            //****************************************
//            _rsy = gamepad2.right_stick_y;
//            _armRotate.setRotationNormalMode(_rsy);
//
//            //****************************************
//            // Arm slide
//            //****************************************
//            _lsy = gamepad2.left_stick_y;;
//            _armSlide.setSlideNormalMode(_lsy);

            isDone = true;

        }
    }

    private void initRobot() throws InterruptedException {

        _logger = new ParameterLogger(this, true);
        _hardwareMap = hardwareMap;
        //_configs = new BBConfigurations();
        _armClaw = new BBArmClaw(this, _logger);
        _armRotate = new BBArmRotate(this, _logger);
        _armSlide = new BBArmSlide(this, _logger);

        _maxDriveSpeed = BBConfigurations.MAX_DRIVE_SPEED;
        // Because autonomous reset yaw, don't reset here.
        //_drive = new MecanumDriveNormal(true, BBConfigurations.getHubOrientation(), this, _logger);
        _drive = new MecanumDrivePrecise(false, BBConfigurations.getHubOrientation(), this, _logger);

        //_vision = new BBVision(false, false, this, _logger);

    }
}
