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




@Autonomous(name="BlueAutonomous", group ="Autonomous")
public class BlueAutonomous extends LinearOpMode {

    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    @Override
    public void runOpMode() throws InterruptedException {

            robot.init(hardwareMap);
            //robot.beaconSensor.enableLed(false);
            //robot.lineSensor.enableLed(true);
            waitForStart();

            robot.frontRightMotor.setTargetPosition(5000);
            robot.frontLeftMotor.setTargetPosition(5000);
            while (robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){
                robot.frontRightMotor.setPower(PID.EncoderPID(robot.frontRightMotor.getTargetPosition(), robot.frontRightMotor.getCurrentPosition(), 0.0001, 0.099));
                robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
                robot.frontLeftMotor.setPower(PID.EncoderPID(robot.frontLeftMotor.getTargetPosition(), robot.frontLeftMotor.getCurrentPosition(), 0.0001, 0.099));
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

            robot.frontRightMotor.setTargetPosition(5000);
            robot.frontLeftMotor.setTargetPosition(5000);
            while (robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){
                robot.frontRightMotor.setPower(PID.EncoderPID(robot.frontRightMotor.getTargetPosition(), robot.frontRightMotor.getCurrentPosition(), 0.0001, 0.099));
                robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
                robot.frontLeftMotor.setPower(PID.EncoderPID(robot.frontLeftMotor.getTargetPosition(), robot.frontLeftMotor.getCurrentPosition(), 0.0001, 0.099));
                robot.backLeftMotor.setPower(robot.frontLeftMotor.getPower());
                telemetry.addData("Encoder Value", robot.frontRightMotor.getCurrentPosition());
                telemetry.update();
                sleep(1);
            }

            eReset();
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

