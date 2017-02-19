package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;


@TeleOp (name = "Color Sensor AutonomousTest", group = "Teleop")
//@Disabled
public class ColorSensorTest extends LinearOpMode{

    HardwareMaelstromBot robot = new HardwareMaelstromBot();

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("beacon sensor alpha", robot.beaconSensor.alpha());
            telemetry.addData("line sensor alpha", robot.lineSensor.alpha());
            telemetry.addData("beacon sensor red", robot.beaconSensor.red());
            telemetry.addData("beacon sensor blue", robot.beaconSensor.blue());
            telemetry.update();

        }


    }

}
