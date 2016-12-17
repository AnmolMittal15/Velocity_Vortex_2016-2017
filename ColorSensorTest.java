package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Ramsey on 11/22/2016.
 */

@TeleOp (name = "Color Sensor AutonomousTest", group = "Teleop")
//@Disabled
public class ColorSensorTest extends LinearOpMode{

    public void runOpMode() {

        ColorSensor beaconSensor = hardwareMap.colorSensor.get("beacon sensor");
        ColorSensor lineSensor = hardwareMap.colorSensor.get("line sensor");

        beaconSensor.setI2cAddress(I2cAddr.create7bit(0x1e));
        lineSensor.setI2cAddress(I2cAddr.create7bit(0x26));

        beaconSensor.enableLed(false);
        lineSensor.enableLed(true);
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("beacon sensor alpha", beaconSensor.alpha());
            telemetry.addData("line sensor alpha", lineSensor.alpha());
            telemetry.addData("beacon sensor red", beaconSensor.red());
            telemetry.addData("beacon sensor blue", beaconSensor.blue());
            telemetry.update();

        }


    }

}
