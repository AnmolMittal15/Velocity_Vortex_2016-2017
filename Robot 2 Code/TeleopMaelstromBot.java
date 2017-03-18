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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name= "Telop Drive")

public class TeleopMaelstromBot extends OpMode implements Runnable {

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
    double secondsSinceStart;
    long startTime;
    long buttonPressedStart;
    //boolean autoshootEnabled = false;
    //boolean slowMode = false;
    //boolean drawReleased = false;
    boolean halfSpeed = false;
    boolean firstLoop = true;
    boolean armsNotDeployed = true;
    boolean threadNotStarted = true;
    double timeSincheHalf = 0.5;
    //double period = 1;



    public void init() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
/*
        Thread LEDStrip = new Thread();
        LEDStrip.start();
*/
        robot.beaconServo.setPower(0);
        robot.indexer.setPosition(0);
        robot.rightGripper.setPosition(0.999);
        robot.leftGripper.setPosition(.001);

        robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.flywheelMotor.setPower(0);

        robot.topLeftIntake.setPower(0);
        robot.bottomLeftIntake.setPower(0);
        robot.topRightIntake.setPower(0);
        robot.bottomRightIntake.setPower(0);

        robot.leftLiftMotor.setPower(0);
        robot.rightLiftTopMotor.setPower(0);
        robot.rightLiftBottomMotor.setPower(0);
/*
        robot.beaconServo.setPosition(0.38);

        robot.leftgripper.setPosition(1);
        robot.rightgripper.setPosition(0);

        robot.drawServoRight.setPosition(0);
        robot.drawServoLeft.setPosition(1);
        */
        telemetry.addLine("Hey");
    }

    public void loop() {
/*
        if (gamepad2.dpad_up) {
            period += 0.001;
        } else if (gamepad2.dpad_down) {
            period -= 0.001;
        }*/

        if (firstLoop) {
            startTime = System.nanoTime();
        }

        if (gamepad1.y && timeSincheHalf >= 0.5) {
            buttonPressedStart = System.nanoTime();
        }
        timeSincheHalf = (System.nanoTime() - buttonPressedStart) / 1e9;
        if (timeSincheHalf > 0.5 && gamepad1.y) {
            halfSpeed = !halfSpeed;
        }

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

        if (gamepad1.x || gamepad2.dpad_right) {
            robot.beaconServo.setPower(1);
        } else if (gamepad1.b || gamepad2.dpad_left) {
            robot.beaconServo.setPower(-1);
        } else {
            robot.beaconServo.setPower(0);
        }

        /*
        if (gamepad2.right_stick_y !=0) {
            robot.rightGripper.setPosition(gamepad2.right_stick_y);
            robot.leftGripper.setPosition(1 - gamepad2.right_stick_y);
        }
        else {
            robot.rightGripper.setPosition(0.999);
            robot.leftGripper.setPosition(.001);
        }
        telemetry.addData("Position:", gamepad2.right_stick_y);
        */
        if (gamepad2.y) { //Up
            robot.rightGripper.setPosition(.93);
            robot.leftGripper.setPosition(.07);
        } else if (gamepad2.x) { //Grip
            robot.rightGripper.setPosition(0.963);
            robot.leftGripper.setPosition(0.037);
            armsNotDeployed = false;
        }
        else if (gamepad2.a) {
            robot.rightGripper.setPosition(0.999);
            robot.leftGripper.setPosition(.001);
        }
        if (gamepad2.right_bumper) {
            robot.leftLiftMotor.setPower(1);
            robot.rightLiftTopMotor.setPower(-1);
            robot.rightLiftBottomMotor.setPower(-1);
        } else if (gamepad2.left_bumper) {
            robot.leftLiftMotor.setPower(-1);
            robot.rightLiftTopMotor.setPower(1);
            robot.rightLiftBottomMotor.setPower(1);
        } else if (gamepad2.right_trigger > 0) {
            robot.leftLiftMotor.setPower(gamepad2.right_trigger);
            robot.rightLiftTopMotor.setPower(-gamepad2.right_trigger);
            robot.rightLiftBottomMotor.setPower(-gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0) {
            robot.leftLiftMotor.setPower(-gamepad2.left_trigger);
            robot.rightLiftTopMotor.setPower(gamepad2.left_trigger);
            robot.rightLiftBottomMotor.setPower(gamepad2.left_trigger);
        } else {
            robot.leftLiftMotor.setPower(0);
            robot.rightLiftTopMotor.setPower(0);
            robot.rightLiftBottomMotor.setPower(0);
        }

        if (gamepad1.right_trigger > 0) {
            robot.flywheelMotor.setPower(1);
        } else if (gamepad1.left_trigger > 0) {
            robot.flywheelMotor.setPower(-1);
        } else {
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

        if (gamepad1.a || gamepad2.b) {
            robot.indexer.setPosition(0.45);
        } else {
            robot.indexer.setPosition(0);
        }
        if (gamepad2.a) {
            Thread LEDRun = new Thread();
            LEDRun.start();

        }
/*
        telemetry.addData("Distance", robot.rangeFinder.cmOptical());
        telemetry.addData("Distance", robot.rangeFinder.cmUltrasonic());
        telemetry.addData("Distance", robot.rangeFinder.rawOptical());
        telemetry.addData("Distance", robot.rangeFinder.getDistance(DistanceUnit.CM));explode
        telemetry.addData("Distance", robot.rangeFinder.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance", robot.rangeFinder.getDistance(DistanceUnit.METER));
        telemetry.addData("Distance", robot.rangeFinder.getDistance(DistanceUnit.MM));
        */

        secondsSinceStart = (System.nanoTime() - startTime) / 1e9;

        if (armsNotDeployed) {
            robot.LEDStrip1.setPower((Math.sin(6 * secondsSinceStart) + 1.05) / 2);
            robot.LEDStrip2.setPower((Math.sin(6 * secondsSinceStart) + 1.05) / 2);
            //telemetry.addData("Period:", period);
        } else if (threadNotStarted){
            Thread LEDOscillator = new Thread();
            LEDOscillator.start();
            threadNotStarted = false;
        }

        firstLoop = false;
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

        telemetry.addData("angle:", angle*180/Math.PI);

        speedMagnitude = Math.hypot(x, y);
        frontLeft = -(Math.sin(angle + (Math.PI/4))) * speedMagnitude + gamepad1.right_stick_x;
        backLeft = -(Math.cos(angle + (Math.PI/4))) * speedMagnitude + gamepad1.right_stick_x;
        frontRight = (Math.cos(angle + (Math.PI/4))) * speedMagnitude + gamepad1.right_stick_x;
        backRight = (Math.sin(angle + (Math.PI/4))) * speedMagnitude + gamepad1.right_stick_x;


        telemetry.addData("x" , gamepad1.right_stick_x);


        driveScaleFactor = Math.abs(Math.max(
                Math.max(frontLeft, frontRight),
                Math.max(backLeft, backRight)))
                != 0 ? Math.abs(Math.max(
                Math.max(frontLeft, frontRight),
                Math.max(backLeft, backRight))) : 1
        ;


        frontLeft /= driveScaleFactor;
        frontRight /= driveScaleFactor;
        backLeft /= driveScaleFactor;
        backRight /= driveScaleFactor;

        telemetry.addData("FrontLeft: ", frontLeft);
        telemetry.addData("FrontRight: ", frontRight);
        telemetry.addData("BackLeft: ", backLeft);
        telemetry.addData("BackRight: ", backRight);

        if (halfSpeed) {
            robot.frontLeftMotor.setPower(0.5*frontLeft);
            robot.backLeftMotor.setPower(0.5*backLeft);
            robot.frontRightMotor.setPower(0.5*frontRight);
            robot.backRightMotor.setPower(0.5*backRight);
        }
        else {
            robot.frontLeftMotor.setPower(frontLeft);
            robot.backLeftMotor.setPower(backLeft);
            robot.frontRightMotor.setPower(frontRight);
            robot.backRightMotor.setPower(backRight);
        }

    }
    public void run() {
        while (true) {
            robot.LEDStrip1.setPower(0);
            robot.LEDStrip2.setPower(1);

            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
            }

            robot.LEDStrip1.setPower(1);
            robot.LEDStrip2.setPower(0);

            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
            }
        }
    }
}
