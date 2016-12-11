package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ramsey on 10/15/2016.
 */

public class HardwareMaelstromBot {

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor liftMotor;
    public DcMotor intakeMotor;
    //public DcMotor flywheelMotor;
    public Servo beaconServo;
    public Servo drawServoRight;
    public Servo drawServoLeft;
    public Servo leftgripper;
    public Servo rightgripper;
    public ColorSensor lineSensor;
    public ColorSensor beaconSensor;
    ModernRoboticsI2cGyro gyro;


    HardwareMap hwMap;


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        frontLeftMotor = hwMap.dcMotor.get("front left motor");
        backLeftMotor = hwMap.dcMotor.get("back left motor");
        frontRightMotor = hwMap.dcMotor.get("front right motor");
        backRightMotor = hwMap.dcMotor.get("back right motor");
        liftMotor = hwMap.dcMotor.get("lift motor");
        intakeMotor = hwMap.dcMotor.get("intake motor");
        //flywheelMotor = hwMap.dcMotor.get("flywheel motor");
        beaconServo = hwMap.servo.get("beacon servo");
        drawServoRight = hwMap.servo.get("draw servo right");
        drawServoLeft = hwMap.servo.get("draw servo left");
        leftgripper = hwMap.servo.get("left gripper servo");
        rightgripper = hwMap.servo.get("right gripper servo");
        lineSensor = hwMap.colorSensor.get("line sensor");
        beaconSensor = hwMap.colorSensor.get("beacon sensor");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("MR gyro");


        beaconSensor.setI2cAddress(I2cAddr.create7bit(0x1e));
        lineSensor.setI2cAddress(I2cAddr.create7bit(0x26));

        beaconSensor.enableLed(false);
        lineSensor.enableLed(true);
    }

}
