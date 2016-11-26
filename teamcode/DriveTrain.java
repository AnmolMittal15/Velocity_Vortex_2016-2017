
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveTrain{
    //HardwareMap hardwareMap;

    //AdafruitIMU imu = new AdafruitIMU("IMU", hardwareMap);
    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    //robot.init(hardwareMap);
    PID PID = new PID();
    public void EncoderDrive(int frontRightEncoder, int frontLeftEncoder, double kp, double ki) {

        eReset();

        robot.frontRightMotor.setTargetPosition(frontRightEncoder);
        robot.frontLeftMotor.setTargetPosition(frontLeftEncoder);

        while (robot.frontRightMotor.isBusy() && robot.frontLeftMotor.isBusy()){

            robot.frontRightMotor.setPower(PID.EncoderPID(robot.frontRightMotor.getTargetPosition(), robot.frontRightMotor.getCurrentPosition(), kp, ki));
            robot.frontLeftMotor.setPower(PID.EncoderPID(robot.frontLeftMotor.getTargetPosition(), robot.frontLeftMotor.getCurrentPosition(), kp, ki));

            robot.backRightMotor.setPower(robot.frontRightMotor.getPower());
            robot.backLeftMotor.setPower(robot.frontLeftMotor.getPower());

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
/*
    public void AngleDrive(int angleTarget, double kp, double ki) {
        double[] angles = imu.getAngles();
        double yaw = angles[0];
        angleTarget += yaw;
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
