
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name= "Telop Drive")

public class TeleopMaelstromBot extends LinearOpMode {

    HardwareMaelstromBot robot = new HardwareMaelstromBot();

    public void runOpMode() throws InterruptedException {
        double left;
        //double right;

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {


            left = gamepad1.left_stick_y;
            //right = gamepad1.right_stick_y;
            robot.motor.setPower(left);
            //robot.frontLeftMotor.setPower(left);
            //robot.backLeftMotor.setPower(left);
            //robot.frontRightMotor.setPower(right);
            //robot.backRightMotor.setPower(right);
/*
            if (gamepad1.a)
                robot.beaconServo.setPosition(0);
            else if (gamepad1.b)
                robot.beaconServo.setPosition(1);
            else robot.beaconServo.setPosition(0.5);

            if (gamepad1.x)
                robot.drawServo.setPosition(0);
            else if (gamepad1.y)
                robot.drawServo.setPosition(1);

            if (gamepad1.right_bumper)
                robot.liftServo.setPosition(0);
*/
            sleep(1);
            idle();
        }
    }
}
