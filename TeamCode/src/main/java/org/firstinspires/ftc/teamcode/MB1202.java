
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@SuppressWarnings({"WeakerAccess", "unused"})
@I2cSensor(name = "MB1202 Ultrasonic", description = "MaxBotix Ultrasonic Sensor", xmlTag = "MB1202")
public class MB1202 extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    public final static I2cAddr ADDRESS_I2C_DEFAULT_8B_W = I2cAddr.create8bit(224);
    public final static I2cAddr ADDRESS_I2C_DEFAULT_8B_R = I2cAddr.create8bit(225);
    public final static I2cAddr ADDRESS_I2C_DEFAULT_7B = I2cAddr.create7bit(112);
    ElapsedTime runtime;
    public enum Register
    {
        FIRST(0),
        WRITE(0x00),
        READ(0x00),
        LAST(READ.bVal),
        UNKNOWN(-1);

        public byte bVal;
        Register(int bVal) { this.bVal = (byte)bVal; }
    }

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected static final int cmUltrasonicMax = 750;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public MB1202(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT_8B_W);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow()
    {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        // write8(Register.WRITE, (byte) 0x51);
        runtime = new ElapsedTime();
        //this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT_8B_W);
        //write8(Register.WRITE, (byte) 0x51);
        //write8(Register.WRITE, (byte) 0x00);
        //this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT_8B_R);
        runtime.reset();
        return true;
    }

    public double getDistance(DistanceUnit unit)
    {
        double cm = cmUltrasonic();
        if (cm == cmUltrasonicMax)
        {
            return DistanceSensor.distanceOutOfRange;
        }
        return unit.fromUnit(DistanceUnit.CM, cm);
    }

    //----------------------------------------------------------------------------------------------
    // Raw sensor data
    //----------------------------------------------------------------------------------------------

    /**
     * Returns the raw reading on the ultrasonic sensor
     * @return the raw reading on the ultrasonic sensor
     */
    public double cmUltrasonic() {
        /*
        if (runtime.milliseconds()>100.0) { // write every 100 ms
            //this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT_8B_W);
            write8(Register.WRITE, (byte) 0x51);
            write8(Register.WRITE, (byte) 0x00);
            runtime.reset();
            //this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT_8B_R);
        }
        */
        short range_word = readShort(Register.READ); //Read 2-bytes
        double range =  (range_word >> 8) * 256 + (range_word & 0xff);
        return range;
    }


    //----------------------------------------------------------------------------------------------
    // I2cAddressConfig
    //----------------------------------------------------------------------------------------------

    public void setI2cAddress(I2cAddr newAddress)
    {
        this.deviceClient.setI2cAddress(newAddress);
    }

    public I2cAddr getI2cAddress()
    {
        return this.deviceClient.getI2cAddress();
    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override public String getDeviceName()
    {
        // return String.format(Locale.getDefault(), "MaxBotix Ultrasonic Sensor");
        return "MaxBotix MB1202 Ultrasonic Sensor";
        //return "mb_ultra";
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    public byte read8(Register reg)
    {
        return this.deviceClient.read8(reg.bVal);
    }

    public void write8(Register reg, byte value)
    {
        this.write8(reg, value, I2cWaitControl.NONE);
    }
    public void write8(Register reg, byte value, I2cWaitControl waitControl)
    {
        this.deviceClient.write8(reg.bVal, value, waitControl);
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    protected int readUnsignedByte(Register reg)
    {
        return TypeConversion.unsignedByteToInt(this.read8(reg));
    }

}
