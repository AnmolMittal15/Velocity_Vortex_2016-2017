package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ramsey on 10/15/2016.
 */

public class HardwareMaelstromBot
{

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor leftLiftMotor;
    public DcMotor rightLiftMotor;
    public DcMotor flywheelMotor;
    //public DcMotor LEDStrip;
    public CRServo topLeftIntake;
    public CRServo bottomLeftIntake;
    public CRServo topRightIntake;
    public CRServo bottomRightIntake;
    public CRServo beaconServo;
    public Servo leftGripper;
    public Servo rightGripper;
    public Servo indexer;
    public ColorSensor lineSensor;
    public ColorSensor beaconSensor;
    public ModernRoboticsI2cRangeSensor rangeFinder;

    HardwareMap hwMap;


    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        frontLeftMotor = hwMap.dcMotor.get("front left motor");
        backLeftMotor = hwMap.dcMotor.get("back left motor");
        frontRightMotor = hwMap.dcMotor.get("front right motor");
        backRightMotor = hwMap.dcMotor.get("back right motor");
        leftLiftMotor = hwMap.dcMotor.get("left lift motor");
        rightLiftMotor = hwMap.dcMotor.get("right lift motor");
        flywheelMotor = hwMap.dcMotor.get("flywheel motor");
        //LEDStrip = hwMap.dcMotor.get("LED Strip");
        topLeftIntake = hwMap.crservo.get("top left intake");
        bottomLeftIntake = hwMap.crservo.get("bottom left intake");
        topRightIntake = hwMap.crservo.get("top right intake");
        bottomRightIntake = hwMap.crservo.get("bottom right intake");
        beaconServo = hwMap.crservo.get("beacon servo");
        leftGripper = hwMap.servo.get("left gripper");
        rightGripper = hwMap.servo.get("right gripper");
        indexer = hwMap.servo.get("indexer");
        lineSensor = hwMap.colorSensor.get("line sensor");
        beaconSensor = hwMap.colorSensor.get("beacon sensor");
        rangeFinder = hwMap.get(ModernRoboticsI2cRangeSensor.class, "Range Finder");


        beaconSensor.setI2cAddress(I2cAddr.create7bit(0x26));
        lineSensor.setI2cAddress(I2cAddr.create7bit(0x1e));

        beaconSensor.enableLed(false);
        lineSensor.enableLed(true);

        //LEDStrip.setPower(1);

    }

}
