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

@Autonomous(name="Shooting and Ball Red", group ="Autonomous")
//@Disabled
public class ShootingAndBallAutoRed extends LinearOpMode {
    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        eReset();

        robot.beaconServo.setPower(0);
        robot.indexer.setPosition(-1);
        robot.rightGripper.setPosition(0.999);
        robot.leftGripper.setPosition(.001);
        /*

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
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);

        double angleTarget;
        double[] angles = imu.getAngles();
        double yaw = angles[0];

        while (!isStarted()) {
            telemetry.addData("Status", "Initialization Complete");
            telemetry.update();
            sleep(10);
        }
        waitForStart();


        telemetry.clear();

        sleep(5000);

        robot.frontLeftMotor.setPower(.6);
        robot.backLeftMotor.setPower(-1);
        robot.frontRightMotor.setPower(.6);
        robot.backRightMotor.setPower(-1);

        sleep(1350);

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        shoot();


        angleTarget = /*-27.5*/ -65;
        PID.i = 0;
        long startTime = System.nanoTime();
        long stopState = 0;
        long startTime2 = System.nanoTime();
        long currentLoopTime = 0;
        while (opModeIsActive() && (stopState <= 1000) && currentLoopTime < 4500) {
            angles = imu.getAngles();
            yaw = angles[0];
            robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, yaw, 0.008, 0.00000000064));
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(robot.frontRightMotor.getPower());
            telemetry.addData("Yaw:", yaw);
            telemetry.addData("Random", Math.random());
            telemetry.update();
            if (yaw >= (angleTarget - 1.5) && yaw <= (angleTarget + 1.5)) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            currentLoopTime = (System.nanoTime() - startTime2)/1000000;
            sleep(1);
        }

        EncoderDrive(/*5750*/ 2750, 0, /*0.00034*/0.00045, 0.00000000049, imu);

    }


    void shoot() {

        robot.flywheelMotor.setPower(1);

        sleep(1000);

        robot.flywheelMotor.setPower(0);

        robot.indexer.setPosition(0.25);

        sleep(1000);

        robot.indexer.setPosition(0);

        sleep(500);

        robot.flywheelMotor.setPower(1);

        sleep(1000);

        robot.flywheelMotor.setPower(0);

    }

    public void EncoderDrive(int rightEncoder, /*int leftEncoder,*/ double angle, double KP, double KI, AdafruitIMU imu) {

        Thread retract = new Thread();
        retract.start();


        //AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        double frontLeft;
        double backLeft;
        double frontRight;
        double backRight;

        eReset();

        double error;
        long startTime = System.nanoTime();
        long stopState = 0;
        rightEncoder = -rightEncoder;
        PID.i =0;
        double initialHeading = imu.getAngles()[0];
        double corrKP = 0.01;
        angle = angle*(Math.PI/180);
        frontLeft = -(Math.sin(angle + (Math.PI/4)));
        backLeft = -(Math.cos(angle + (Math.PI/4)));
        frontRight = (Math.cos(angle + (Math.PI/4)));
        backRight = (Math.sin(angle + (Math.PI/4)));

        while (opModeIsActive() && (stopState <= 1000)) {
            error = imu.getAngles()[0] - initialHeading;
            //telemetry.addData("Integral:", PID.i);

            robot.frontLeftMotor.setPower((frontLeft * PID.EncoderPID(rightEncoder, robot.frontRightMotor.getCurrentPosition(), KP, KI)) + (corrKP * error));
            robot.backLeftMotor.setPower((backLeft * PID.EncoderPID(rightEncoder, robot.frontRightMotor.getCurrentPosition(), KP, KI)) /*+ (corrKP * error)*/);
            robot.frontRightMotor.setPower((frontRight * PID.EncoderPID(rightEncoder, robot.frontRightMotor.getCurrentPosition(), KP, KI)) /*+ (corrKP * error)*/);
            robot.backRightMotor.setPower((backRight * PID.EncoderPID(rightEncoder, robot.frontRightMotor.getCurrentPosition(), KP, KI)) + (corrKP * error));

            //telemetry.addData("Encoder Error:", PID.tempError);

/*
            robot.frontRightMotor.setPower(PID.EncoderPID(encoder, robot.frontRightMotor.getCurrentPosition(), KP, KI));
            robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower() + corrKP * error);
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(-robot.frontRightMotor.getPower() + corrKP * error);
*/
            //telemetry.addData("Front Left Encoder",robot.frontLeftMotor.getCurrentPosition());
            //telemetry.addData("Back Left Encoder",robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Encoder",robot.frontRightMotor.getCurrentPosition());
            //telemetry.addData("Back Left Encoder",robot.backLeftMotor.getCurrentPosition());
            //telemetry.addData("Back Right Encoder",robot.backRightMotor.getCurrentPosition());

            telemetry.addData("Front Right Power",robot.frontRightMotor.getPower());
            /*
            telemetry.addData("Back Right Power",robot.backRightMotor.getPower());
            telemetry.addData("Front Left Power",robot.frontLeftMotor.getPower());
            telemetry.addData("Back Left Power",robot.backLeftMotor.getPower());
            */
            //telemetry.addData("kI", KI);
            telemetry.addData("PID:", PID.EncoderPID(rightEncoder, robot.frontRightMotor.getCurrentPosition(), KP, KI));
            telemetry.addData("Error:", PID.tempError);
            telemetry.update();

            if ((robot.frontRightMotor.getCurrentPosition() >= (rightEncoder - 50)) &&
                    (robot.frontRightMotor.getCurrentPosition() <= (rightEncoder + 50)) /*&&
                (robot.backLeftMotor.getCurrentPosition() >= (leftEncoder - 50)) &&
                (robot.backLeftMotor.getCurrentPosition() <= (leftEncoder + 50))*/) {
                stopState = (System.nanoTime() - startTime)/1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            sleep(1);
        }

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        eReset();
    }

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