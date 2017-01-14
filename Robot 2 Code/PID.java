package org.firstinspires.ftc.teamcode;

/**
 * Created by London on 10/15/2016.
 */

public class PID
{
    double i = 0;

    public double EncoderPID(double target, double currentLoc, double kp, double ki)
    {
        double error = target - currentLoc;
        i += error;
        double power = (kp*error) + (ki*i);

        if (power > 0.8) {
            power = 0.8;
        }

        if (power < -0.8) {
            power = -0.8;
        }

        return power;
    }

    public double AnglePID(double target, double currentLoc, double kp, double ki)
    {
        double error = target - currentLoc;
        i += error;
        double power = (kp*error) + (ki*i);

        if (power > 0.8) {
            power = 0.8;
        }

        if (power < -0.8) {
            power = -0.8;
        }

        return power;
    }
}
