/**

 This is Archi:

 O/
 /|
 / \

 If you code like Archi,

 then your code will not work.

 Do not code like Archi.

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by davjun on 10/30/2016.
 */

@Autonomous(name="Beacon Test", group ="Autonomous")

public class BeaconTest extends LinearOpMode{
    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.frontLeftMotor.setPower(0.5);
        robot.backLeftMotor.setPower(0.5);
        robot.frontRightMotor.setPower(-0.5);
        robot.backRightMotor.setPower(-0.5);

        sleep(1500);


        while(opModeIsActive() && (robot.lineSensor.alpha() < 20)) {
            telemetry.addData("Front Right Power",robot.frontRightMotor.getPower());
            telemetry.addData("Front Left Power",robot.frontLeftMotor.getPower());
            telemetry.addData("Back Right Power",robot.backRightMotor.getPower());
            telemetry.addData("Back Left Power",robot.backLeftMotor.getPower());
            telemetry.addData("Red",robot.beaconSensor.red());
            telemetry.addData("Blue",robot.beaconSensor.blue());
            telemetry.addData("Line Sensor Alpha",robot.lineSensor.alpha());

            telemetry.update();
            sleep(1);

        }
        sleep(10);

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        sleep(5000);

        if (robot.beaconSensor.red() >= 2) {

            robot.beaconServo.setPosition(0);
            sleep(300);
        }

        else if (robot.beaconSensor.blue() >= 2) {

            robot.beaconServo.setPosition(0.75);
            sleep(300);
        }

    }


    public void BeaconChecker() {
        while(opModeIsActive() && (robot.lineSensor.alpha() < 20)) {
            telemetry.addData("Front Right Power",robot.frontRightMotor.getPower());
            telemetry.addData("Front Left Power",robot.frontLeftMotor.getPower());
            telemetry.addData("Back Right Power",robot.backRightMotor.getPower());
            telemetry.addData("Back Left Power",robot.backLeftMotor.getPower());
            telemetry.addData("Red",robot.beaconSensor.red());
            telemetry.addData("Blue",robot.beaconSensor.blue());
            telemetry.addData("Line Sensor Alpha",robot.lineSensor.alpha());

            telemetry.update();
            sleep(1);

        }
        sleep(10);

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        sleep(5000);

        if (robot.beaconSensor.red() >= 2) {

            robot.beaconServo.setPosition(0);
            sleep(300);
        }

        else if (robot.beaconSensor.blue() >= 2) {

            robot.beaconServo.setPosition(0.75);
            sleep(300);
        }
    }
    /*
        public void EncoderDrive(int distance, double kp, double ki) {
            eReset();
            int encoder = distance * -20;
            PID.i = 0;
            while (opModeIsActive()) {
                robot.frontRightMotor.setPower(PID.EncoderPID(encoder, robot.frontRightMotor.getCurrentPosition(), kp, ki));
                robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower());
                robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
                robot.backLeftMotor.setPower(-robot.frontRightMotor.getPower());
                telemetry.addData("Right Encoder",robot.frontRightMotor.getCurrentPosition());
                telemetry.addData("Front Right Power",robot.frontRightMotor.getPower());
                telemetry.addData("Front Left Power",robot.frontLeftMotor.getPower());
                telemetry.addData("Back Right Power",robot.backRightMotor.getPower());
                telemetry.addData("Back Left Power",robot.backLeftMotor.getPower());
                telemetry.update();

                sleep(1);
            }
            eReset();
        }
    */
    void eReset() {

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
