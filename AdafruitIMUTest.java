/*

This is Archi:

            O/
           /|
           / \

If you code like Archi,

then your code will not work.

Do not code like Archi.

 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by davjun on 11/9/2016.
 */

@TeleOp(name= "Adafruit IMU Test")
//@Disabled
public class AdafruitIMUTest extends LinearOpMode {
//fuck rzmmyramham

    public void runOpMode() throws InterruptedException {
        // in this demo, the IMU is named "IMU" in the robot configuration file
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);

        // wait to see this on the Driver Station before pressing play, to make sure the IMU has been initialized
        while (!isStarted()) {
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
        }

        waitForStart();
        telemetry.clear();

        while (opModeIsActive()) {
            double[] angles = imu.getAngles();
            double yaw = angles[0];
            double pitch = angles[1];
            double roll = angles[2];

            telemetry.addData(imu.getName(), imu.telemetrize());
            telemetry.update();
        }
    }

}