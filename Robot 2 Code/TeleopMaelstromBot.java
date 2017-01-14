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
    double frontLeft;
    double backLeft;
    double frontRight;
    double backRight;
    double angle;
    double speedMagnitude;
    double driveScaleFactor;
    double x;
    double y;
    double startTime;
    boolean autoshootEnabled = false;
    boolean slowMode = false;
    boolean drawReleased = false;

    public void init() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        robot.beaconServo.setPower(0);
        robot.indexer.setPosition(0);
        robot.rightGripper.setPosition(0);
        robot.leftGripper.setPosition(1);
/*
        robot.beaconServo.setPosition(0.38);

        robot.leftgripper.setPosition(1);
        robot.rightgripper.setPosition(0);

        robot.drawServoRight.setPosition(0);
        robot.drawServoLeft.setPosition(1);
        */
    }

    public void loop() {

        if (gamepad1.right_bumper) {
            robot.topLeftIntake.setPower(-1);
            robot.bottomLeftIntake.setPower(-1);
            robot.topRightIntake.setPower(1);
            robot.bottomRightIntake.setPower(1);
        } else if (gamepad1.left_bumper) {
            robot.topLeftIntake.setPower(1);
            robot.bottomLeftIntake.setPower(1);
            robot.topRightIntake.setPower(-1);
            robot.bottomRightIntake.setPower(-1);
        } else {
            robot.topLeftIntake.setPower(0);
            robot.bottomLeftIntake.setPower(0);
            robot.topRightIntake.setPower(0);
            robot.bottomRightIntake.setPower(0);
        }

              drive();

        if (gamepad1.x) {
             robot.beaconServo.setPower(1);
        }
        else if (gamepad1.b) {
            robot.beaconServo.setPower(-1);
        }
        else {
            robot.beaconServo.setPower(0);
        }

        if (gamepad2.y) {
            robot.rightGripper.setPosition(1);
            robot.leftGripper.setPosition(0);
        }
        else if (gamepad2.x) {
            robot.rightGripper.setPosition(0.5);
            robot.leftGripper.setPosition(0.5);
        }
        else if (gamepad2.a) {
            robot.rightGripper.setPosition(0);
            robot.leftGripper.setPosition(1);
        }
        if (gamepad2.right_bumper) {
            robot.leftLiftMotor.setPower(1);
            robot.rightLiftMotor.setPower(-1);
        }
        else if (gamepad2.left_bumper) {
            robot.leftLiftMotor.setPower(-1);
            robot.rightLiftMotor.setPower(1);
        }
        else {
            robot.leftLiftMotor.setPower(0);
            robot.rightLiftMotor.setPower(0);
        }

        if (gamepad1.right_trigger > 0) {
            robot.flywheelMotor.setPower(1);
        }
        else if (gamepad1.left_trigger >0) {
            robot.flywheelMotor.setPower(-1);
        }
        else {
            robot.flywheelMotor.setPower(0);
        }
        /*else if (!autoshootEnabled){
            robot.flywheelMotor.setPower(0);
        }*/

        /*if (autoshootEnabled && (System.nanoTime() - startTime)/1e9 >= 1000 && (System.nanoTime() - startTime)/1e9 < 1500) {
            robot.flywheelMotor.setPower(1);
            robot.indexer.setPosition(1);
        }
        else if ((System.nanoTime() - startTime)/1e9 >= 1500) {
            robot.flywheelMotor.setPower(0);
            autoshootEnabled = false; //FTFY
        }

        if (gamepad1.a && !autoshootEnabled) {
            robot.indexer.setPosition(0.5);
            autoshootEnabled = true;
            startTime = System.nanoTime();
        }*/

        if (gamepad1.a) {
            robot.indexer.setPosition(0.25);
        }
        else {
            robot.indexer.setPosition(0);
        }

    }

    public void drive() {
        x = gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;
        if (x != 0) {
            angle = Math.atan(y/x);
        }
        else {
            angle = 0;
        }
        if (x < 0 && y > 0) {
            angle = angle + Math.PI;
        }
        else if (x < 0 && y <= 0) {
            angle = angle + Math.PI;
        }
        else if (x > 0 && y < 0) {
            angle = angle + (2*Math.PI);
        }
        else if (x == 0 && y > 0 ) {
            angle = Math.PI/2;
        }
        else if (x == 0 && y < 0 ) {
            angle = (3 * Math.PI) / 2;
        }

        speedMagnitude = Math.hypot(x, y);
        frontLeft = -speedMagnitude*(Math.sin(angle + (Math.PI/4))) + gamepad1.right_stick_x;
        backLeft = -speedMagnitude*(Math.cos(angle + (Math.PI/4))) + gamepad1.right_stick_x;
        frontRight = speedMagnitude*(Math.cos(angle + (Math.PI/4))) + gamepad1.right_stick_x;
        backRight = speedMagnitude*(Math.sin(angle + (Math.PI/4))) + gamepad1.right_stick_x;

        driveScaleFactor = Math.max(
                Math.max(frontLeft, frontRight),
                Math.max(backLeft, backRight)
        );

        if (driveScaleFactor < 1) driveScaleFactor = 1;

        frontLeft *= driveScaleFactor;
        frontRight *= driveScaleFactor;
        backLeft *= driveScaleFactor;
        backRight *= driveScaleFactor;

        robot.frontLeftMotor.setPower(frontLeft);
        robot.backLeftMotor.setPower(backLeft);
        robot.frontRightMotor.setPower(frontRight);
        robot.backRightMotor.setPower(backRight);

    }
}
