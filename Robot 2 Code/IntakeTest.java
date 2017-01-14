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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name= "Intake Test")

public class IntakeTest extends OpMode {

    HardwareMaelstromBot robot = new HardwareMaelstromBot();


    public void init() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.addData("Say", "London is better than Ramsey");
        telemetry.update();
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
        }
        else if (gamepad1.left_bumper) {
            robot.topLeftIntake.setPower(1);
            robot.bottomLeftIntake.setPower(1);
            robot.topRightIntake.setPower(-1);
            robot.bottomRightIntake.setPower(-1);
        }
        else {
            robot.topLeftIntake.setPower(0);
            robot.bottomLeftIntake.setPower(0);
            robot.topRightIntake.setPower(0);
            robot.bottomRightIntake.setPower(0);
        }
    }
}

