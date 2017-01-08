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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "Grabber Test")
//@Disabled
public class GrabberTest extends OpMode {
/*
public Servo grabberRight;
public Servo grabberLeft;
*/
public DcMotor rightMotor;
public DcMotor leftMotor;

    public void init() {
/*
        grabberRight = hardwareMap.servo.get("Grabber Right");
        grabberLeft = hardwareMap.servo.get("Grabber Left");
        */
        rightMotor = hardwareMap.dcMotor.get("Right Motor");
        leftMotor = hardwareMap.dcMotor.get("Left Motor");

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
/*
        grabberLeft.setPosition(0);
        grabberRight.setPosition(1);
        */
    }

    public void loop() {
/*
        if (gamepad1.a) {

            grabberLeft.setPosition(1);
            grabberRight.setPosition(0);
        }
        else {
            grabberLeft.setPosition(0);
            grabberRight.setPosition(1);
        }
*/
        if (gamepad1.right_bumper) {
            rightMotor.setPower(.15);
            leftMotor.setPower(-.15);
        }

        if (gamepad1.left_bumper) {
            rightMotor.setPower(-.15);
            leftMotor.setPower(.15);
        }

        if (gamepad1.a) {
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }

    }
}

