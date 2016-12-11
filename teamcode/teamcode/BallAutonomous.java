package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by davjun on 10/30/2016.
 */

@Autonomous(name="Ball Autonomous", group ="Autonomous")
@Disabled
public class BallAutonomous extends LinearOpMode{

    HardwareMaelstromBot robot = new HardwareMaelstromBot();


    @Override
    public void runOpMode() {

        waitForStart();
        robot.init(hardwareMap);

        sleep(10000);

        robot.frontLeftMotor.setPower(1);
        robot.backLeftMotor.setPower(1);
        robot.frontRightMotor.setPower(-1);
        robot.backRightMotor.setPower(-1);

        sleep(1600);

    }

}


