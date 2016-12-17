package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;




@Autonomous(name="RedAutonomousOld", group ="Autonomous")
@Disabled
public class RedAutonomousOld extends LinearOpMode {

    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    //DriveTrain DriveTrain = new DriveTrain();

    @Override
    public void runOpMode() throws InterruptedException {
            AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
            robot.init(hardwareMap);
            robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            //robot.beaconSensor.enableLed(false);
            //robot.lineSensor.enableLed(true);
            waitForStart();


            EncoderDrive(1000, 1000, 0.001, 0);
        /*
            double angleTarget = 45;
            double[] angles = imu.getAngles();
            double yaw = angles[0];
            angleTarget += yaw;
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (yaw >= (angleTarget - 1) && yaw <= (angleTarget + 1)) {
                yaw = angles[0];
                robot.frontRightMotor.setPower(PID.AnglePID(angleTarget, yaw, 0.01, 0));
                robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower());
                robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
                robot.backLeftMotor.setPower(-robot.frontLeftMotor.getPower());

                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        */
            //DriveTrain.AngleDrive(45, 0.0001, 0);
            EncoderDrive(1000, 1000, 0.001, 0);


/*
            robot.frontRightMotor.setTargetPosition(1000);
            robot.frontLeftMotor.setTargetPosition(1000);
            while (robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){
                robot.frontRightMotor.setPower(PID.EncoderPID(robot.frontRightMotor.getTargetPosition(), robot.frontRightMotor.getCurrentPosition(), 0.0001, 0));
                robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
                robot.frontLeftMotor.setPower(PID.EncoderPID(robot.frontLeftMotor.getTargetPosition(), robot.frontLeftMotor.getCurrentPosition(), 0.0001, 0));
                robot.backLeftMotor.setPower(robot.frontLeftMotor.getPower());
                telemetry.addData("Encoder Value", robot.frontRightMotor.getCurrentPosition());
                telemetry.update();
                sleep(1);
            }

            eReset();

            robot.frontRightMotor.setTargetPosition(-1470);
            robot.frontLeftMotor.setTargetPosition(1470);
            while (robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){
                robot.frontRightMotor.setPower(PID.EncoderPID(robot.frontRightMotor.getTargetPosition(), robot.frontRightMotor.getCurrentPosition(), 0.0001, 0));
                robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
                robot.frontLeftMotor.setPower(PID.EncoderPID(robot.frontLeftMotor.getTargetPosition(), robot.frontLeftMotor.getCurrentPosition(), 0.0001, 0));
                robot.backLeftMotor.setPower(robot.frontLeftMotor.getPower());
                telemetry.addData("Encoder Value", robot.frontRightMotor.getCurrentPosition());
                telemetry.update();
                sleep(1);
            }

            eReset();

            robot.frontRightMotor.setTargetPosition(1000);
            robot.frontLeftMotor.setTargetPosition(1000);
            while (robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){
                robot.frontRightMotor.setPower(PID.EncoderPID(robot.frontRightMotor.getTargetPosition(), robot.frontRightMotor.getCurrentPosition(), 0.0001, 0));
                robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
                robot.frontLeftMotor.setPower(PID.EncoderPID(robot.frontLeftMotor.getTargetPosition(), robot.frontLeftMotor.getCurrentPosition(), 0.0001, 0));
                robot.backLeftMotor.setPower(robot.frontLeftMotor.getPower());
                telemetry.addData("Encoder Value", robot.frontRightMotor.getCurrentPosition());
                telemetry.update();
                sleep(1);
            }

            eReset();
*/
/*
            if (robot.lineSensor.argb() >= 180 && robot.lineSensor.argb() < 300) {

                robot.frontLeftMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backRightMotor.setPower(0);

                if (robot.beaconSensor.red() >= 2)

                    robot.beaconServo.setPosition(0);

                else if (robot.beaconSensor.blue() >= 2)

                    robot.beaconServo.setPosition(0.75);
            }
*/
    }
/*
    void eReset() {
        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

public void AngleDrive(int angleTarget, double kp, double ki) {
    double[] angles = imu.getAngles();
    double yaw = angles[0];
    angleTarget += yaw;
    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    while (yaw >= (angleTarget-1) && yaw <= (angleTarget+1)){
        yaw = angles[0];
        robot.frontRightMotor.setPower(PID.AnglePID(angleTarget, yaw, kp, ki));
        robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower());
        robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
        robot.backLeftMotor.setPower(-robot.frontLeftMotor.getPower());

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
*/
public void EncoderDrive(int frontRightEncoder, int frontLeftEncoder, double kp, double ki) {

    eReset();

    robot.frontRightMotor.setTargetPosition(frontRightEncoder);
    robot.frontLeftMotor.setTargetPosition(frontLeftEncoder);

    while (robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){

        robot.frontRightMotor.setPower(PID.EncoderPID(robot.frontRightMotor.getTargetPosition(), robot.frontRightMotor.getCurrentPosition(), kp, ki));
        robot.frontLeftMotor.setPower(PID.EncoderPID(robot.frontLeftMotor.getTargetPosition(), robot.frontLeftMotor.getCurrentPosition(), kp, ki));
        telemetry.addData("Left Encoder",robot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("Right Encoder",robot.frontRightMotor.getCurrentPosition());
        telemetry.update();
        robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
        robot.backLeftMotor.setPower(robot.frontLeftMotor.getPower());

        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
    void eReset() {

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}

