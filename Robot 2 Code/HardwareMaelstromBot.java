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
    public DcMotor rightLiftTopMotor;
    public DcMotor rightLiftBottomMotor;
    public DcMotor flywheelMotor;
    public DcMotor LEDStrip1;
    public DcMotor LEDStrip2;
    public CRServo topLeftIntake;
    public CRServo bottomLeftIntake;//rammyramham is a nerd
    public CRServo topRightIntake;
    public CRServo bottomRightIntake;
    public CRServo beaconServo;
    public Servo leftGripper;
    public Servo rightGripper;
    public Servo indexer;
    public ColorSensor lineSensor;
    public ColorSensor beaconSensor;
    //public ModernRoboticsI2cRangeSensor rangeFinder;

    HardwareMap hwMap;


    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;

        frontLeftMotor = hwMap.dcMotor.get("front left motor");
        backLeftMotor = hwMap.dcMotor.get("back left motor");
        frontRightMotor = hwMap.dcMotor.get("front right motor");
        backRightMotor = hwMap.dcMotor.get("back right motor");
        leftLiftMotor = hwMap.dcMotor.get("left lift motor");
        rightLiftTopMotor = hwMap.dcMotor.get("right lift top motor");
        rightLiftBottomMotor = hwMap.dcMotor.get("right lift bottom motor");
        flywheelMotor = hwMap.dcMotor.get("flywheel motor");
        LEDStrip1 = hwMap.dcMotor.get("LED Strip 1");
        LEDStrip2 = hwMap.dcMotor.get("LED Strip 2");
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
        //rangeFinder = hwMap.get(ModernRoboticsI2cRangeSensor.class, "Range Finder");


        beaconSensor.setI2cAddress(I2cAddr.create7bit(0x26));
        lineSensor.setI2cAddress(I2cAddr.create7bit(0x1e));

        beaconSensor.enableLed(false);
        lineSensor.enableLed(true);

        //LEDStrip.setPower(1);

    }

}
