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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by davjun on 10/30/2016.
 */

@Autonomous(name="Blue Autonomous", group ="Autonomous")
//@Disabled
public class BlueAuto extends LinearOpMode{

    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    boolean redDetected = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        eReset();

        robot.beaconServo.setPower(0);
        robot.indexer.setPosition(-1);
        robot.rightGripper.setPosition(0.999);
        robot.leftGripper.setPosition(.001);
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
/*
        EncoderDrive(325, 0, 0.0005, 0.00000000052);

        angleTarget = 27.5;
        PID.i = 0;
        long startTime = System.nanoTime();
        long stopState = 0;
        while (opModeIsActive() && (stopState <= 1000)) {
            angles = imu.getAngles();
            yaw = angles[0];
            robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, yaw, 0.0067, 0.00000000043));
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(robot.frontRightMotor.getPower());
            telemetry.addData("Yaw:", yaw);
            telemetry.addData("Random", Math.random());
            telemetry.update();
            if (yaw >= (angleTarget - 0.5) && yaw <= (angleTarget + 0.5)) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            sleep(1);
        }
        */
        //EncoderDrive(8180, 7650, 315, 0.000093, 0.0000000005);

        EncoderDrive(/*5750*/ 5600, 180, 0.00034, 0.00000000045);

        angleTarget = /*-27.5*/ 24;
        PID.i = 0;
        long startTime = System.nanoTime();
        long stopState = 0;
        while (opModeIsActive() && (stopState <= 1000)) {
            angles = imu.getAngles();
            yaw = angles[0];
            robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, yaw, 0.0072, 0.00000000053));
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(robot.frontRightMotor.getPower());
            telemetry.addData("Yaw:", yaw);
            telemetry.addData("Random", Math.random());
            telemetry.update();
            if (yaw >= (angleTarget - 1) && yaw <= (angleTarget + 1)) {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else {
                startTime = System.nanoTime();
            }
            sleep(1);
        }

        BeaconChecker(false);

        sleep(1000);

        BeaconChecker(true);
        shoot();

    }
/*

    }
/*
    public void AngleDrive(int angleTarget, double KP, double KI) {
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        //angleTarget += yaw;
        PID.i = 0;
        while (opModeIsActive() && (yaw <= (angleTarget - 2) || yaw >= (angleTarget + 2))) {
            angles = imu.getAngles();
            yaw = angles[0];
            robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, yaw, KP, KI));
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
*/

    void shoot() {

        if (redDetected) {

            robot.frontLeftMotor.setPower(0.3);
            robot.backLeftMotor.setPower(-1);
            robot.frontRightMotor.setPower(0.3);
            robot.backRightMotor.setPower(-1);

            sleep(1125);

            robot.frontLeftMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backRightMotor.setPower(0);

        }
        else {
            robot.frontLeftMotor.setPower(0.375);
            robot.backLeftMotor.setPower(-1);
            robot.frontRightMotor.setPower(0.375);
            robot.backRightMotor.setPower(-1);

            sleep(750);

            robot.frontLeftMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backRightMotor.setPower(0);
        }
/*
        if (redDetected) {
            AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
            double angleTarget = 20;
            long startTime = System.nanoTime();
            long stopState = 0;
            while (opModeIsActive() && (stopState <= 1000)) {
                double[] angles = imu.getAngles();
                double yaw = angles[0];
                robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, yaw, 0.0082, 0));
                robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
                robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
                robot.backLeftMotor.setPower(robot.frontRightMotor.getPower());
                telemetry.addData("Yaw:", yaw);
                telemetry.addData("Random", Math.random());
                telemetry.update();
                if (yaw >= (angleTarget - 2) && yaw <= (angleTarget + 2)) {
                    stopState = (System.nanoTime() - startTime) / 1000000;
                } else {
                    startTime = System.nanoTime();
                }
                sleep(1);
            }
            sleep (10000);
        }
*/
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

    public void EncoderDrive(int rightEncoder, /*int leftEncoder,*/ double angle, double KP, double KI) {
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
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
            telemetry.addData("Integral:", PID.i);

            robot.frontLeftMotor.setPower((frontLeft * PID.EncoderPID(rightEncoder, robot.frontRightMotor.getCurrentPosition(), KP, KI)) + (corrKP * error));
            robot.backLeftMotor.setPower((backLeft * PID.EncoderPID(rightEncoder, robot.backRightMotor.getCurrentPosition(), KP, KI)) /*+ (corrKP * error)*/);
            robot.frontRightMotor.setPower((frontRight * PID.EncoderPID(rightEncoder, robot.frontRightMotor.getCurrentPosition(), KP, KI)) /*+ (corrKP * error)*/);
            robot.backRightMotor.setPower((backRight * PID.EncoderPID(rightEncoder, robot.frontRightMotor.getCurrentPosition(), KP, KI)) + (corrKP * error));

/*
            robot.frontRightMotor.setPower(PID.EncoderPID(encoder, robot.frontRightMotor.getCurrentPosition(), KP, KI));
            robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower() + corrKP * error);
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(-robot.frontRightMotor.getPower() + corrKP * error);
*/
            //telemetry.addData("Front Left Encoder",robot.frontLeftMotor.getCurrentPosition());
            //telemetry.addData("Back Left Encoder",robot.backLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Encoder",robot.frontRightMotor.getCurrentPosition());
            telemetry.addData("Back Left Encoder",robot.backLeftMotor.getCurrentPosition());
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

    public void BeaconChecker(boolean reverse) {
        /*
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        double initialHeading = imu.getAngles()[0];
        double error;
        */

        eReset();

        //double timeKP = 1.5;
        long startTime;
        double elapsedTime;
        double leftPower;
        double rightPower;
        double corrKP = 0.01;

        if (!reverse) {
            robot.frontLeftMotor.setPower(-.06);
            robot.backLeftMotor.setPower(-.06);
            robot.frontRightMotor.setPower(.06);
            robot.backRightMotor.setPower(.06);

            while(opModeIsActive() && (robot.lineSensor.alpha() <= 50)) {
                //error = imu.getAngles()[0] - initialHeading;
                //telemetry.addData("Error:", error);
                /*
                robot.frontLeftMotor.setPower(robot.frontLeftMotor.getPower() + (corrKP * error));
                robot.backRightMotor.setPower(robot.backRightMotor.getPower() + (corrKP * error));
                */
                telemetry.addData("Front Left Power",robot.frontLeftMotor.getPower());
                telemetry.addData("Back Left Power",robot.backLeftMotor.getPower());
                telemetry.addData("Front Right Power",robot.frontRightMotor.getPower());
                telemetry.addData("Back Right Power",robot.backRightMotor.getPower());

                telemetry.addData("Line Sensor Alpha",robot.lineSensor.alpha());
                telemetry.update();
                sleep(1);
            }
        }
        else {

            robot.frontLeftMotor.setPower(.15);
            robot.backLeftMotor.setPower(.15);
            robot.frontRightMotor.setPower(-.1625);
            robot.backRightMotor.setPower(-.1625);

            //startTime = System.nanoTime();
            while((opModeIsActive() && (robot.lineSensor.alpha() <= 50)) || robot.frontRightMotor.getCurrentPosition() <= 1000) {
                /*
                elapsedTime = (System.nanoTime() - startTime)/1e9;
                telemetry.addData("ElapsedTime:", elapsedTime);

                leftPower = -0.75+(timeKP*elapsedTime);
                rightPower = 0.75-(timeKP*elapsedTime);

                if (rightPower < 0.15) {
                    rightPower = 0.15;
                }

                if (leftPower > -0.15) {
                    leftPower = -0.15;
                }

                robot.frontLeftMotor.setPower(leftPower);
                robot.backLeftMotor.setPower(leftPower);
                robot.frontRightMotor.setPower(rightPower);
                robot.backRightMotor.setPower(rightPower);
                //error = imu.getAngles()[0] - initialHeading;
                //telemetry.addData("Error:", error);
                */
                /*
                robot.frontLeftMotor.setPower(robot.frontLeftMotor.getPower() + (corrKP * error));
                robot.backLeftMotor.setPower(robot.backLeftMotor.getPower() + (corrKP * error));
                robot.frontRightMotor.setPower(robot.frontRightMotor.getPower() + (corrKP * error));
                robot.backRightMotor.setPower(robot.backRightMotor.getPower() + (corrKP * error));
*/

                telemetry.addData("Front Left Power",robot.frontLeftMotor.getPower());
                telemetry.addData("Back Left Power",robot.backLeftMotor.getPower());
                telemetry.addData("Front Right Power",robot.frontRightMotor.getPower());
                telemetry.addData("Back Right Power",robot.backRightMotor.getPower());

                telemetry.addData("Line Sensor Alpha",robot.lineSensor.alpha());
                telemetry.update();
                sleep(1);
            }
        }

        telemetry.addLine("Line Detected!");

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        /*
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        sleep(500);

        /*
        robot.frontLeftMotor.setPower(-.2);
        robot.backLeftMotor.setPower(.2);
        robot.frontRightMotor.setPower(-.2);
        robot.backRightMotor.setPower(.2);

        telemetry.clear();

        telemetry.addData("beacon sensor red", robot.beaconSensor.red());
        telemetry.addData("beacon sensor blue", robot.beaconSensor.blue());

        while (opModeIsActive() && (robot.beaconSensor.blue() == robot.beaconSensor.red())) {
            telemetry.addData("beacon sensor red", robot.beaconSensor.red());
            telemetry.addData("beacon sensor blue", robot.beaconSensor.blue());
            sleep (10);
        }

        sleep(200);

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        */

        if (robot.beaconSensor.blue() > robot.beaconSensor.red()){

            telemetry.addLine("Blue Side Detected!");
            telemetry.update();

            robot.frontRightMotor.setPower(-.1);
            robot.frontLeftMotor.setPower(.1);
            robot.backRightMotor.setPower(-.1);
            robot.backLeftMotor.setPower(.1);

            if (reverse) {
                sleep(775);
            }
            else {
                sleep(575);
            }

            robot.frontLeftMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            /*
            robot.frontRightMotor.setPower(PID.EncoderPID(-75, robot.frontRightMotor.getCurrentPosition(), 0.004, 0));
            robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower());
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(-robot.frontRightMotor.getPower());

            startTime = System.nanoTime();
            stopState = 0;

            while (opModeIsActive() && (stopState <= 500)) {
                if ((robot.frontRightMotor.getCurrentPosition() >= (-75 - 10)) &&
                        (robot.frontRightMotor.getCurrentPosition() <= (-75 + 10))) {
                    stopState = (System.nanoTime() - startTime) / 1000000;
                } else {
                    startTime = System.nanoTime();
                }
            }
            */

            robot.beaconServo.setPower(1);

            sleep(2750);

            robot.beaconServo.setPower(-1);

            sleep(2750);

            robot.beaconServo.setPower(0);

            //new Presser().retract();

            //sleep(500);
        }

        else if (robot.beaconSensor.red() > robot.beaconSensor.blue() ) {

            telemetry.addLine("Red Side Detected!");
            telemetry.update();

            if (reverse) {
                redDetected = true;
            }

            robot.frontRightMotor.setPower(.1);
            robot.frontLeftMotor.setPower(-.1);
            robot.backRightMotor.setPower(.1);
            robot.backLeftMotor.setPower(-.1);

            if (!reverse) {
                sleep(560);
            }
            else {
                sleep(325);
            }

            robot.frontLeftMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            /*
            robot.frontRightMotor.setPower(PID.EncoderPID(75, robot.frontRightMotor.getCurrentPosition(), 0.004, 0));
            robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower());
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(-robot.frontRightMotor.getPower());

            startTime = System.nanoTime();
            stopState = 0;

            while (opModeIsActive() && (stopState <= 500)) {
                if ((robot.frontRightMotor.getCurrentPosition() >= (75 - 10)) &&
                        (robot.frontRightMotor.getCurrentPosition() <= (75 + 10))) {
                    stopState = (System.nanoTime() - startTime) / 1000000;
                } else {
                    startTime = System.nanoTime();
                }
            }
            */

            robot.beaconServo.setPower(1);

            sleep(2750);

            robot.beaconServo.setPower(-1);

            sleep(2750);

            robot.beaconServo.setPower(0);

            //new Presser().retract();

            //sleep(500);


        }

        eReset();

    }
    void eReset() {

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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