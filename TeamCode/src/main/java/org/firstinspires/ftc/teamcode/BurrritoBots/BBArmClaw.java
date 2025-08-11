package org.firstinspires.ftc.teamcode.BurrritoBots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.ParameterLogger;

public class BBArmClaw {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private ParameterLogger _logger;
    private BBConfigurations _configs;

    // Expansion Hub:
    //    Servo port0: "clawServo"
    private Servo _clawServo;
    private double _clawPosition;

    private String _paramClawPosition = "Claw Position";

    public BBArmClaw(LinearOpMode opMode, ParameterLogger logger) throws InterruptedException{
        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _logger = logger;

        init();
    }

    private void init() throws InterruptedException {
        _configs = new BBConfigurations();
        _clawServo = _hardwareMap.get(Servo.class, "clawServo"); //--> servo port 0

        // Add all of the parameters you want to see on the driver hub display.
        _logger.addParameter(_paramClawPosition);
    }

    // set variables for max claw extensions in ERCGlobalConfig
    public void setClawOpen() {
        _clawServo.setPosition(0.0);
        _clawPosition = _clawServo.getPosition();
        _logger.updateParameter(_paramClawPosition, _clawPosition);
    }
    public void setClawClose() {
        _clawServo.setPosition(_configs.MAX_CLAW_POSITION);
        _clawPosition = _clawServo.getPosition();
        _logger.updateParameter(_paramClawPosition, _clawPosition);
    }
}
