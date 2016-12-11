package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by davjun on 10/30/2016.
 */

@Autonomous(name="Defensive Ball Autonomous", group ="Autonomous")
@Disabled
public class DefensiveBallAutonomous extends LinearOpMode{

    HardwareMaelstromBot robot = new HardwareMaelstromBot();
    PID PID = new PID();

    @Override
    public void runOpMode() {

        waitForStart();
        robot.init(hardwareMap);

        sleep(10000);

        EncoderDrive(10000, 0.0003, 0);

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

    void eReset() {

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}


