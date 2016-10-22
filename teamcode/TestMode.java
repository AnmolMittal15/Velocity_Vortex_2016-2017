package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import android.app.Activity;
import android.graphics.Color;
import android.media.MediaPlayer;
import android.view.View;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="TestMode")
@Disabled
public class TestMode extends LinearOpMode {


    PID PID = new PID();
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor;
        motor = hardwareMap.dcMotor.get("motor");

        waitForStart();
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(10000);

        while (motor.isBusy()){
            motor.setPower(PID.PID(motor.getTargetPosition(), motor.getCurrentPosition(), 0.0001, 0.099));
            telemetry.addData("Encoder Value", motor.getCurrentPosition());
            telemetry.update();
            sleep(1);
        }
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setTargetPosition(10000);

        while (motor.isBusy()){
            motor.setPower(PID.PID(motor.getTargetPosition(), motor.getCurrentPosition(), 0.0001, 0.099));
            telemetry.addData("Encoder Value", motor.getCurrentPosition());
            telemetry.update();
            sleep(1);
        }

    }

}

