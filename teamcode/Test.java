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

@Autonomous(name="AutonomousTestTESTserzdfxerztgrxgxrthgdrtyhtrTESTESTEST", group ="Autonomous")
@Disabled
public class Test extends LinearOpMode{
    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        /*
        AngleDrive(45, 0.03, 0);
        EncoderDrive(24, 0.0015, 0);
        AngleDrive(-45, 0.03, 0);

        robot.frontLeftMotor.setPower(0.75);
        robot.backLeftMotor.setPower(0.75);
        robot.frontRightMotor.setPower(0.75);
        robot.backRightMotor.setPower(0.75);
*/
        telemetry.clear();

        BeaconChecker();

        robot.frontLeftMotor.setPower(0.75);
        robot.backLeftMotor.setPower(0.75);
        robot.frontRightMotor.setPower(0.75);
        robot.backRightMotor.setPower(0.75);

        BeaconChecker();

        AngleDrive(-160, 0.03, 0);
        EncoderDrive(50, 0.03, 0);
        EncoderDrive(-8, 0.03, 0);
    }

    public void AngleDrive(int angleTarget, double kp, double ki) {
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        angleTarget += yaw;
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
            if ((robot.frontRightMotor.getCurrentPosition() >= (encoder - 10)) && (robot.frontRightMotor.getCurrentPosition() <= (encoder + 10))) {
                stopState = System.nanoTime() - startTime;
            }
            else {
                startTime = System.nanoTime();
            }

            sleep(1);
        }
        eReset();
    }

    public void BeaconChecker() {
        while(opModeIsActive() /*&& (robot.lineSensor.argb() <= 180 || robot.lineSensor.argb() > 300)*/) {
            telemetry.addData("Front Right Power",robot.frontRightMotor.getPower());
            telemetry.addData("Front Left Power",robot.frontLeftMotor.getPower());
            telemetry.addData("Back Right Power",robot.backRightMotor.getPower());
            telemetry.addData("Back Left Power",robot.backLeftMotor.getPower());
            telemetry.addData("Hue",robot.lineSensor.argb());
            telemetry.addData("Red",robot.lineSensor.red());
            telemetry.addData("Blue",robot.lineSensor.blue());
            telemetry.update();
            /*
            if (robot.lineSensor.argb() >= 180 && robot.lineSensor.argb() < 300) {

                robot.frontLeftMotor.setPower(0);
                robot.backLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.backRightMotor.setPower(0);
                sleep(1000);

                if (robot.beaconSensor.red() >= 2) {

                    robot.beaconServo.setPosition(0);
                    sleep(300);
                }

                else if (robot.beaconSensor.blue() >= 2) {

                    robot.beaconServo.setPosition(0.75);
                    sleep(300);
                }
            }
            */
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


/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name= "Test")
public class Test extends LinearOpMode {
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