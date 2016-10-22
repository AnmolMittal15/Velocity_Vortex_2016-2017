package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ramsey on 10/15/2016.
 */

public class HardwareMaelstromBot {
    public Servo beaconServo;
    public Servo drawServo;
    public Servo liftServo;
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public ColorSensor beaconSensor;
    public ColorSensor lineSensor;
    public DcMotor motor;

    HardwareMap hwMap;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        motor = hwMap.dcMotor.get("motor");

        beaconServo = ahwMap.servo.get("beacon servo");
        frontLeftMotor = hwMap.dcMotor.get("front left motor");
        backLeftMotor = hwMap.dcMotor.get("back left motor");
        frontRightMotor = hwMap.dcMotor.get("front right motor");
        backRightMotor = hwMap.dcMotor.get("back left motor");
        beaconSensor = hwMap.colorSensor.get("beacon sensor");
        lineSensor = hwMap.colorSensor.get("line sensor");

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

}
