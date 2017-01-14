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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by davjun on 10/30/2016.
 */

@Autonomous(name="Autonomous Test With Righting", group ="Autonomous")
//@Disabled
public class RedAutonomousLeft extends LinearOpMode{
    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    double frontLeft;
    double backLeft;
    double frontRight;
    double backRight;
    double angle;
    long startTime;
    long stopState;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        eReset();

        robot.beaconServo.setPower(0);
        robot.indexer.setPosition(-1);
        robot.rightGripper.setPosition(0);
        robot.leftGripper.setPosition(1);
/*

        robot.beaconServo.setPosition(0.38);

        robot.leftgripper.setPosition(1);
        robot.rightgripper.setPosition(0);

        robot.drawServoRight.setPosition(0);
        robot.drawServoLeft.setPosition(1);
*/
        telemetry.addData("Status", "Initialization Starting...");
        telemetry.update();
        /*
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);

        int angleTarget;
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        */

/*
        while (!isStarted()) {
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
            sleep(10);
        }
*/
        waitForStart();


        telemetry.clear();

        EncoderDrive(4000, 45, 0.001, 0);

        robot.flywheelMotor.setPower(1);

        sleep(500);

        robot.flywheelMotor.setPower(0);

        shoot();

/*
        robot.topLeftIntake.setPower(-1);
        robot.bottomLeftIntake.setPower(-1);
        robot.topRightIntake.setPower(1);
        robot.bottomRightIntake.setPower(1);

        EncoderDrive(450, 0, 0.0006, 0);

        sleep(3000);

        robot.topLeftIntake.setPower(0);
        robot.bottomLeftIntake.setPower(0);
        robot.topRightIntake.setPower(0);
        robot.bottomRightIntake.setPower(0);

        EncoderDrive(450, 180, 0.0006, 0);

        shoot();
*/


/*
            angleTarget = 90;
            PID.i = 0;
            startTime = System.nanoTime();
            stopState = 0;
            while (opModeIsActive() && (stopState <= 1000)) {
                angles = imu.getAngles();
                yaw = angles[0];
                robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, yaw, 0.0082, 0));
                robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
                robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
                robot.backLeftMotor.setPower(0);
                telemetry.addData("Yaw:", yaw);
                telemetry.addData("Random", Math.random());
                telemetry.update();
                if (yaw >= (angleTarget - 2) && yaw <= (angleTarget + 2)) {
                    stopState = (System.nanoTime() - startTime) / 1000000;
                }
                else {
                    startTime = System.nanoTime();
                }
                sleep(1);
            }

        BeaconChecker(false);

        BeaconChecker(true);
        */
/*
        angleTarget = -180;
        PID.i = 0;
        while (opModeIsActive() && (yaw <= (angleTarget - 2) || yaw >= (angleTarget + 2))) {
            angles = imu.getAngles();
            yaw = angles[0];
            robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, yaw, 0.008, 0));
            robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(robot.frontRightMotor.getPower());
            telemetry.addData("Yaw:", yaw);
            yaw = angles[0];
            sleep(1);
        }

        EncoderDrive(24, 0.0003, 0);
        */
    }

    public void AngleDrive(int angleTarget, double kp, double ki) {
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        //angleTarget += yaw;
        PID.i = 0;
        while (opModeIsActive() && (yaw <= (angleTarget - 2) || yaw >= (angleTarget + 2))) {
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

    void shoot() {

        robot.indexer.setPosition(0.25);

        sleep(1000);

        robot.indexer.setPosition(0);

        sleep(500);

        robot.flywheelMotor.setPower(1);

        sleep(1000);

        robot.flywheelMotor.setPower(0);
    }

    public void EncoderDrive(int encoder, double angle, double kp, double ki) {
        /*
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        eReset();
        long startTime = System.nanoTime();
        long stopState = 0;
        encoder = -encoder;
        PID.i =0;
        double initialHeading = imu.getAngles()[0];
        double corrKP = 0.0035;
*/

        frontLeft = -(Math.sin(angle + (Math.PI/4)));
        backLeft = -(Math.cos(angle + (Math.PI/4)));
        frontRight = (Math.cos(angle + (Math.PI/4)));
        backRight = (Math.sin(angle + (Math.PI/4)));

        while (opModeIsActive() && (stopState <= 1000)) {
            //double error = imu.getAngles()[0] - initialHeading;
            telemetry.addData("Integral:", PID.i);

            robot.frontLeftMotor.setPower(frontLeft * PID.EncoderPID(encoder, robot.frontRightMotor.getCurrentPosition(), kp, ki));
            robot.backLeftMotor.setPower(backLeft * PID.EncoderPID(encoder, robot.frontRightMotor.getCurrentPosition(), kp, ki));
            robot.frontRightMotor.setPower(frontRight * PID.EncoderPID(encoder, robot.frontRightMotor.getCurrentPosition(), kp, ki));
            robot.backRightMotor.setPower(backRight * PID.EncoderPID(encoder, robot.frontRightMotor.getCurrentPosition(), kp, ki));

/*
            robot.frontRightMotor.setPower(PID.EncoderPID(encoder, robot.frontRightMotor.getCurrentPosition(), kp, ki));
            robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower() + corrKP * error);
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(-robot.frontRightMotor.getPower() + corrKP * error);
*/
            telemetry.addData("Front Left Encoder",robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("Back Left Encoder",robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Encoder",robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("Back Right Encoder",robot.backRightMotor.getCurrentPosition());
            telemetry.addData("Front Left Power",robot.frontLeftMotor.getPower());
            telemetry.addData("Back Left Power",robot.backLeftMotor.getPower());
            telemetry.addData("Front Right Power",robot.frontRightMotor.getPower());
            telemetry.addData("Back Right Power",robot.backRightMotor.getPower());
            //telemetry.addData("kI", ki);
            telemetry.update();

            if ((robot.frontRightMotor.getCurrentPosition() >= (encoder - 100)) &&
                (robot.frontRightMotor.getCurrentPosition() <= (encoder + 100))) {
                stopState = (System.nanoTime() - startTime)/1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            sleep(1);
        }
        eReset();
    }
/*
    public void BeaconChecker(boolean reverse) {
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        double initialHeading = imu.getAngles()[0];
        double corrKP = 0.002;
        if (!reverse) {
            robot.frontLeftMotor.setPower(0.15);
            robot.backLeftMotor.setPower(0.15);
            robot.frontRightMotor.setPower(-0.15);
            robot.backRightMotor.setPower(-0.15);
        }
        else {
            robot.frontLeftMotor.setPower(-0.15);
            robot.backLeftMotor.setPower(-.15);
            robot.frontRightMotor.setPower(0.15);
            robot.backRightMotor.setPower(0.15);
        }

        while(opModeIsActive() && (robot.lineSensor.alpha() != 255)) {
            double error = imu.getAngles()[0] - initialHeading;
            robot.frontLeftMotor.setPower(robot.frontLeftMotor.getPower() + corrKP * error);
            robot.backLeftMotor.setPower(robot.backLeftMotor.getPower() + corrKP * error);
            telemetry.addData("Line Sensor Alpha",robot.lineSensor.alpha());
            telemetry.update();
            sleep(1);
        }
        if (robot.lineSensor.alpha() == 255) {
            telemetry.addLine("Line Detected!");
            telemetry.update();
        }

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        if (robot.beaconSensor.blue() >= 3) {

            telemetry.addLine("Blue Side Detected!");
            telemetry.update();
            robot.beaconServo.setPosition(0);
        }

        else if (robot.beaconSensor.red() >= 3) {

            telemetry.addLine("Red Side Detected!");
            telemetry.update();
            robot.beaconServo.setPosition(0.75);

        }
        sleep(1000);
        robot.beaconServo.setPosition(0.38);
    }
*/
    void eReset() {

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}


/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name= "AutonomousTest")
public class AutonomousTest extends LinearOpMode {
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
        double[] angles = imu.getAngles();
        double yaw = 67;

        while (opModeIsActive()) {
            // the next 4 lines show how to retrieve angles from the imu and use them
            angles = imu.getAngles();
            yaw = angles[0];
            // this adds telemetry data using the telemetrize() method in the MasqAdafruitIMU class
            telemetry.addData("Yaw:" , yaw);
            telemetry.update();
        }
    }

}
*/