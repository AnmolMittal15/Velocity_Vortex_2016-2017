package org.firstinspires.ftc.teamcode;

/**
 * Created by Ramsey on 10/15/2016.
 */

public class PID {
    public double EncoderPID(double target, double currentLoc, double kp, double ki) {
        double error = target - currentLoc;
        double i = 0;
        i += error;
        double power = kp*error + ki*i;
        return power;

    }

    public double AnglePID(double target, double currentLoc, double kp, double ki) {
        double error = target - currentLoc;
        double i = 0;
        i += error;
        double power = kp*error + ki*i;
        return power;

    }
}
