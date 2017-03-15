/*
Modern Robotics Range Sensor Example
Created 9/8/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.x Beta
Reuse permitted with credit where credit is due

Configuration:
I2cDevice on an Interface Module named "range" at the default address of 0x28 (0x14 7-bit)

This program can be run without a battery and Power Destitution Module.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

public class MRRangeSensor {

    private final I2cDevice RANGE1;
    private final I2cDeviceSynch RANGE1Reader;

    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    private I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)

    public MRRangeSensor(String name, HardwareMap hwmap) {
        RANGE1 = hwmap.i2cDevice.get(name);
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
    }

    public byte[] getDistance() {
        return RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
    }

}