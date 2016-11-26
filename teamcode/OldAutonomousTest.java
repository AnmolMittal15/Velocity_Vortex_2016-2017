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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by davjun on 10/30/2016.
 */

@Autonomous(name="Old Autonomous AutonomousTest", group ="Autonomous")
@Disabled
public class OldAutonomousTest extends LinearOpMode{
    AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.lineSensor.enableLed(false);


        waitForStart();
        EncoderDrive(20, 0.03, 0);
        AngleDrive(45, 0.03, 0);
        EncoderDrive(20, 0.03, 0);
        AngleDrive(-45, 0.03, 0);
/*
        robot.frontLeftMotor.setPower(0.75);
        robot.backLeftMotor.setPower(0.75);
        robot.frontRightMotor.setPower(0.75);
        robot.backRightMotor.setPower(0.75);


        while(opModeIsActive() && (robot.lineSensor.argb() <= 180 || robot.lineSensor.argb() > 300)) {

            if (robot.lineSensor.argb() >= 180 && robot.lineSensor.argb() < 300) {

                robot.frontLeftMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backRightMotor.setPower(0);

                if (robot.beaconSensor.red() >= 2) {

                    robot.beaconServo.setPosition(0);
                    sleep(300);
                }
                else if (robot.beaconSensor.blue() >= 2) {

                    robot.beaconServo.setPosition(0.75);
                    sleep(300);
                }
            }
        }
        robot.frontLeftMotor.setPower(0.75);
        robot.backLeftMotor.setPower(0.75);
        robot.frontRightMotor.setPower(0.75);
        robot.backRightMotor.setPower(0.75);
        while(opModeIsActive() && (robot.lineSensor.argb() <= 180 || robot.lineSensor.argb() > 300)) {
            if (robot.lineSensor.argb() >= 180 && robot.lineSensor.argb() < 300) {

                robot.frontLeftMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backRightMotor.setPower(0);

                if (robot.beaconSensor.red() >= 2) {

                    robot.beaconServo.setPosition(0);
                    sleep(300);
                }

                else if (robot.beaconSensor.blue() >= 2) {

                    robot.beaconServo.setPosition(0.75);
                    sleep(300);
                }
            }
        }
*/
        AngleDrive(-160, 0.03, 0);
        EncoderDrive(50, 0.03, 0);
        EncoderDrive(-8, 0.03, 0);
    }

    public void AngleDrive(int angleTarget, double kp, double ki) {

        double[] angles = imu.getAngles();
        double yaw = angles[0];
        angleTarget += yaw;
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PID.i = 0;
        while (opModeIsActive() && (yaw <= (angleTarget - 1) || yaw >= (angleTarget + 1))) {
            angles = imu.getAngles();
            yaw = angles[0];
            robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, yaw, kp, ki));
            robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(robot.frontRightMotor.getPower());
            telemetry.addData("Yaw:", yaw);
            telemetry.update();
            angles = imu.getAngles();
            yaw = angles[0];
            sleep(1);
        }
    }

    public void EncoderDrive(int distance, double kp, double ki) {
        eReset();
        long startTime = System.nanoTime();
        long stopState = 0;
        int encoder = distance * -20;
        PID.i = 0;
        while (opModeIsActive() && stopState <= 500) {
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
            if (robot.frontRightMotor.getCurrentPosition() >= encoder - 10 && robot.frontRightMotor.getCurrentPosition() <= encoder + 10) {
                stopState = (System.nanoTime() - startTime)/1000000;
            }
            else {
                stopState = 0;
            }

            sleep(1);
        }
        eReset();
    }

    void eReset() {

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}



