package org.firstinspires.ftc.teamcode.BurrritoBots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Common.ParameterLogger;

public class BBArmSlide {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ParameterLogger _logger;
    private BBConfigurations _configs;

    // Expansion Hub:
    //    Motor port1: "slideMotor"     (GoBILDA 5202/3/4 series)
    private DcMotor _slide = null;
    public int _slideEncoderVal;

    private String _paramSlideEncVal = "Slide Encoder Val";

    public BBArmSlide(LinearOpMode opMode, ParameterLogger logger) throws InterruptedException{
        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        init();
    }

    private void init() throws InterruptedException {
        _configs = new BBConfigurations();
        _slide = _hardwareMap.dcMotor.get("slideMotor"); //--> motor port 1
        reset();

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramSlideEncVal);
    }

    public void reset() {
        _slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //_slide.setDirection(DcMotor.Direction.REVERSE);
        _slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSlideNormalMode(double upPower) {
        _slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _slide.setPower(upPower);
        logCurrentPosition();
    }

    public void setRotationAutoMode(int encoderVal, double power) {
        _slide.setTargetPosition(encoderVal);
        _slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _slide.setPower(Math.abs(power));

        //while (_rotation.isBusy() && !_gamepad2.right_bumper) {
        while (_slide.isBusy()) {
            _logger.writeMsg("waiting for rotation to reach target" + encoderVal);
        }
        //setSlideNormalMode(0); // here so motor stops being busy when done
        logCurrentPosition();
    }

    public void logCurrentPosition() {
        _slideEncoderVal = _slide.getCurrentPosition();
        _logger.updateParameter(_paramSlideEncVal, _slideEncoderVal);
    }
}
