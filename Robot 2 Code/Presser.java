package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by Debby on 2/10/2017.
 */

public class Presser  {

    HardwareMap hwMap;
    public CRServo beaconServo = hwMap.crservo.get("beacon servo");

    public void retract() {
        beaconServo.setPower(-1);

        try {
            Thread.sleep(3500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        beaconServo.setPower(0);
    }
}