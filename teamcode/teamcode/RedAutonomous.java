
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

@Autonomous(name="Red Autonomous", group ="Autonomous")
@Disabled
public class RedAutonomous extends LinearOpMode{

    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.lineSensor.enableLed(false);


        waitForStart();
        EncoderDrive(20, 0.05, 0);
        AngleDrive(45, 0.05, 0);
        EncoderDrive(20, 0.05, 0);
        AngleDrive(-45, 0.05, 0);

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
        AngleDrive(-160, 0.01, 0);
        EncoderDrive(50, 0.001, 0);
        EncoderDrive(-8, 0.001, 0);
    }

    public void AngleDrive(int angleTarget, double kp, double ki) {
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        angleTarget += yaw;
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        int encoder = distance * 45;
        robot.frontRightMotor.setTargetPosition(encoder);
        PID.i = 0;
        while (opModeIsActive() && robot.frontRightMotor.isBusy()) {
            robot.frontRightMotor.setPower(PID.EncoderPID(robot.frontRightMotor.getTargetPosition(), robot.frontRightMotor.getCurrentPosition(), kp, ki));
            robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower());
            telemetry.addData("Right Encoder",robot.frontRightMotor.getCurrentPosition());
            telemetry.update();
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(-robot.frontRightMotor.getPower());

            sleep(1);
        }
    }
    void eReset() {

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}

/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="Red Autonomous", group ="Autonomous")
public class RedAutonomous extends LinearOpMode{

    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.lineSensor.enableLed(false);


        waitForStart();
        AngleDrive(45, 0.01, 0);
        EncoderDrive(84, 0.01, 0);
        AngleDrive(-45, 0.01, 0);

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
        AngleDrive(-160, 0.01, 0);
        EncoderDrive(50, 0.001, 0);
        EncoderDrive(-8, 0.001, 0);
    }

    public void AngleDrive(double angleTarget, double kp, double ki) {
        AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        angleTarget += yaw;
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        PID.i = 0;
        while (opModeIsActive() && (yaw <= (angleTarget - 1) || yaw >= (angleTarget + 1))) {
            angles = imu.getAngles();
            yaw = angles[0];
            robot.frontRightMotor.setPower(PID.AnglePID(angleTarget, yaw, kp, ki));
            robot.frontLeftMotor.setPower(robot.frontRightMotor.getPower());
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(robot.frontRightMotor.getPower());
            telemetry.addData("Yaw:", yaw);
            telemetry.update();
            sleep(1);
        }
    }

    public void EncoderDrive(int distance, double kp, double ki){
        eReset();
        int encoder = distance * 45;
        robot.frontRightMotor.setTargetPosition(encoder);
        PID.i = 0;
        while (opModeIsActive() && robot.frontRightMotor.isBusy()){
            robot.frontRightMotor.setPower(PID.EncoderPID(robot.frontRightMotor.getTargetPosition(), robot.frontRightMotor.getCurrentPosition(), kp, ki));
            robot.frontLeftMotor.setPower(-robot.frontRightMotor.getPower());
            telemetry.addData("Left Encoder",robot.frontLeftMotor.getCurrentPosition());
            telemetry.addData("Right Encoder",robot.frontRightMotor.getCurrentPosition());
            telemetry.update();
            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(-robot.frontRightMotor.getPower());

            sleep(1);
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
*/

