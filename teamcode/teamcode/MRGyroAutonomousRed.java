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

@Autonomous(name="Red Autonomous", group ="Autonomous")
@Disabled
public class MRGyroAutonomousRed extends LinearOpMode{
    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialization Starting...");
        telemetry.update();

        while (robot.gyro.isCalibrating())  {
            sleep(50);
        }

        telemetry.addLine("Calibration complete");
        telemetry.update();

        waitForStart();

        telemetry.clear();

        EncoderDrive(400, 0.001, 0);

        AngleDrive(45, 0.03, 0);

        EncoderDrive(3000, 0.0003, 0);

        AngleDrive(0, 0.03, 0);


        robot.frontLeftMotor.setPower(0.75);
        robot.backLeftMotor.setPower(0.75);
        robot.frontRightMotor.setPower(0.75);
        robot.backRightMotor.setPower(0.75);
        telemetry.clear();

        BeaconChecker();

        robot.frontLeftMotor.setPower(0.75);
        robot.backLeftMotor.setPower(0.75);
        robot.frontRightMotor.setPower(0.75);
        robot.backRightMotor.setPower(0.75);

        BeaconChecker();

        //AngleDrive(-160, 0.03, 0);
/*
        angleTarget = -160;
        PID.i = 0;
        while (opModeIsActive() && (yaw <= (angleTarget - 2) || yaw >= (angleTarget + 2))) {
            angles = imu.getAngles();
            yaw = angles[0];
            robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, yaw, 0.025, 0));
            robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(robot.frontRightMotor.getPower());
            telemetry.addData("Yaw:", yaw);
            telemetry.update();
            angles = imu.getAngles();
            yaw = angles[0];
            sleep(1);
        }
*/
        EncoderDrive(24, 0.0003, 0);
    }
    public void AngleDrive(int angleTarget, double kp, double ki) {
        double currentAngle = robot.gyro.getIntegratedZValue();
        while (opModeIsActive() && (currentAngle <= (angleTarget - 2) || currentAngle >= (angleTarget + 2))) {
            robot.frontRightMotor.setPower(-PID.AnglePID(angleTarget, currentAngle, kp, ki));
            robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(robot.frontRightMotor.getPower());
            telemetry.addData("Yaw:", currentAngle);
            telemetry.update();
            sleep(1);
        }
    }
    public void EncoderDrive(int encoder, double kp, double ki) {
        eReset();
        long startTime = System.nanoTime();
        long stopState = 0;
        encoder = -encoder;
        PID.i = 0;
        while (opModeIsActive() && (stopState <= 500)) {
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
            if ((robot.frontRightMotor.getCurrentPosition() >= (encoder - 100)) && (robot.frontRightMotor.getCurrentPosition() <= (encoder + 100))) {
                stopState = (System.nanoTime() - startTime)/1000000;
            }
            else {
                startTime = System.nanoTime();
            }

            sleep(1);
        }
        eReset();
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
