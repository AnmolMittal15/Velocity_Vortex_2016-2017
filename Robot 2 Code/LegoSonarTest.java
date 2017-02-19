package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Sonar Test", group = "Sensor")
@Disabled
public class LegoSonarTest extends LinearOpMode {

    HardwareMaelstromBot robot = new HardwareMaelstromBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            //telemetry.addData("Distance", robot.legacy.readAnalogRaw(5));

            telemetry.update();
        }
    }
}
