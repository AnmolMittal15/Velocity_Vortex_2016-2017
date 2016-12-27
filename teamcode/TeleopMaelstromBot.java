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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Telop Drive")

public class TeleopMaelstromBot extends OpMode {

    HardwareMaelstromBot robot = new HardwareMaelstromBot();

    double left;
    double right;
    boolean slowMode = false;
    boolean drawReleased = false;

    public void init() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        robot.beaconServo.setPosition(0.38);

        robot.leftgripper.setPosition(1);
        robot.rightgripper.setPosition(0);

        robot.drawServoRight.setPosition(0);
        robot.drawServoLeft.setPosition(1);
    }

    public void loop() {

        if (gamepad1.y) {
            slowMode = !slowMode;
            while(gamepad1.y){}
        }

        if (!slowMode) {
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            robot.frontLeftMotor.setPower(-left);
            robot.backLeftMotor.setPower(-left);
            robot.frontRightMotor.setPower(right);
            robot.backRightMotor.setPower(right);
        }
        else {
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            robot.frontLeftMotor.setPower(0.5* -left);
            robot.backLeftMotor.setPower(0.5* -left);
            robot.frontRightMotor.setPower(0.5 * right);
            robot.backRightMotor.setPower(0.5 * right);


                /*
                left = gamepad1.left_stick_y;
                right = gamepad1.right_stick_x;
                if (gamepad1.right_stick_x == 0) {
                    robot.frontLeftMotor.setPower(left);
                    robot.backLeftMotor.setPower(left);
                    robot.frontRightMotor.setPower(left);
                    robot.backRightMotor.setPower(left);
                }
                else if (gamepad1.right_stick_x > 0){
                    robot.frontRightMotor.setPower(left - (0.75 * right));
                    robot.backRightMotor.setPower(left - (0.75 * right));
                }
                    else {
                    robot.frontLeftMotor.setPower(left - (0.75 * right));
                    robot.backLeftMotor.setPower(left - (0.75 * right));
                    }
                    */
        }
        if (gamepad1.a) {
            robot.beaconServo.setPosition(0.75);
        }
        else if (gamepad1.b) {
            robot.beaconServo.setPosition(0);
        }
        else {
            robot.beaconServo.setPosition(0.38);
        }
        if (gamepad2.x) {
            robot.drawServoRight.setPosition(1);
            robot.drawServoLeft.setPosition(0);
            drawReleased = true;
        }
        if (gamepad2.y && drawReleased) {
            robot.leftgripper.setPosition(0.5);
            robot.rightgripper.setPosition(0.5);
        }
        if (gamepad2.right_bumper && drawReleased) {
            robot.liftMotor.setPower(1);
        }
        else if (gamepad2.left_bumper) {
            robot.liftMotor.setPower(-1);
        }
        else {
            robot.liftMotor.setPower(0);
        }
        if (gamepad1.right_bumper) {
            robot.intakeMotor.setPower(1);
        } else if (gamepad1.left_bumper) {
            robot.intakeMotor.setPower(-1);
        }
        else {
            robot.intakeMotor.setPower(0);
        }

    }
}

