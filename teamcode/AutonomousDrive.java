package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class AutonomousDrive {
    HardwareMap hwMap;
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        DcMotor frontLeftMotor;
        DcMotor backLeftMotor;
        DcMotor frontRightMotor;
        DcMotor backRightMotor;


        frontLeftMotor = hwMap.dcMotor.get("front left motor");
        backLeftMotor = hwMap.dcMotor.get("back left motor");
        frontRightMotor = hwMap.dcMotor.get("front right motor");
        backRightMotor = hwMap.dcMotor.get("back right motor");


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setTargetPosition(1800);
        frontRightMotor.setTargetPosition(1800);

        frontLeftMotor.setPower(1);
        backLeftMotor.setPower(1);
        frontRightMotor.setPower(1);
        backRightMotor.setPower(1);



    }





}
