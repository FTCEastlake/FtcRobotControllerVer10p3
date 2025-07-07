package org.firstinspires.ftc.teamcode.BurrritoBots;

import org.firstinspires.ftc.teamcode.Common.*;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BBotNormalDriveTeleOp")
public class BBotNormalDriveTeleOp extends LinearOpMode {

    private Configurations _configs;
    private ParameterLogger _logger;
    private MecanumDriveNormal _drive;
    //private MecanumDrivePrecise _drive;

    private double _maxDriveSpeed;

    private ElapsedTime _debounce = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        while (!isStarted()) {

            // Add code

            _logger.updateStatus("isStarted = false");
            _logger.updateAll();

        }


        //******************************
        // Main loop
        //******************************
        waitForStart();
        _logger.resetCycleTimer();


        _maxDriveSpeed = 0.5;   // override _maxDriveSpeed from default configuration
        while (!isStopRequested()) {

            _logger.updateStatus("isStopRequested = false");
            _logger.updateAll();

            // If you press the start button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.start) {
                _drive.resetYaw();
            }

            // If you press the left bumper, you get a drive from the point of view of the robot
            // (much like driving an RC vehicle)
            if (gamepad1.left_bumper) {
                _drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, _maxDriveSpeed);
            } else {
                _drive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, _maxDriveSpeed);
            }
        }
    }

    private void initRobot() throws InterruptedException {

        _configs = new Configurations();
        _logger = new ParameterLogger(this, true);

        _maxDriveSpeed = _configs.getMaxDriveSpeed();
        // Because autonomous reset yaw, don't reset here.
        _drive = new MecanumDriveNormal(false, _configs.getHubOrientation(), this, _logger);
        //_drive = new MecanumDrivePrecise(false, _configs.getHubOrientation(), this, _logger);

    }
}
