package org.firstinspires.ftc.teamcode.BurrritoBots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.ParameterLogger;

public class BBArmRotate {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ParameterLogger _logger;
    private BBConfigurations _configs;

    // Expansion Hub:
    //    Motor port0: "rotationMotor"  (GoBILDA 5202/3/4 series)
    private DcMotor _rotation = null;
    public int _rotationEncoderVal;

    private String _paramRotateEncVal = "Rotate Encoder Val";

    public BBArmRotate(LinearOpMode opMode, ParameterLogger logger) throws InterruptedException{
        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        init();
    }

    private void init() throws InterruptedException {
        _configs = new BBConfigurations();
        _rotation = _hardwareMap.dcMotor.get("rotationMotor"); //--> motor port 0
        reset();

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramRotateEncVal);
    }

    public void reset() {
        _rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //_rotation.setDirection(DcMotor.Direction.REVERSE);
        _rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setRotationNormalMode(double power) {
        _rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _rotation.setPower(power);
        logCurrentPosition();
    }

    public void setRotationAutoMode(int encoderVal, double power) {
        _rotation.setTargetPosition(encoderVal);
        _rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _rotation.setPower(Math.abs(power));

        //while (_rotation.isBusy() && !_gamepad2.right_bumper) {
        while (_rotation.isBusy()) {
            _logger.writeMsg("waiting for rotation to reach target" + encoderVal);
        }
        //setRotationNormalMode(0); // here so motor stops being busy when done
        logCurrentPosition();
    }

    public void logCurrentPosition() {
        _rotationEncoderVal = _rotation.getCurrentPosition();
        _logger.updateParameter(_paramRotateEncVal, _rotationEncoderVal);
    }
}
