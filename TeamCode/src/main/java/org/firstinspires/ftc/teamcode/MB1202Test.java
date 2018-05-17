/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.PrintWriter;
import java.io.StringWriter;
@Disabled
@TeleOp(name="MB-Test", group ="Test")
public class MB1202Test extends LinearOpMode {

    private MB1202 mb_ultra;
    private Wire mb2;
    boolean use_wire = false;
    boolean use_mb = true;
    boolean use_digit = true;

    DigitalChannel dSen;  // Hardware Device Object
    //private ModernRoboticsI2cRangeSensor mb_ultra;
    private ModernRoboticsI2cRangeSensor mr_range;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        int box_column = -1;
        boolean dSen_st = false;
        /*
        robot.use_swerve = false;
        robot.use_imu = false;
        robot.use_color_sensor = false;
        robot.use_range_sensor = false;
        robot.use_arm = false;
        robot.use_glyph_grabber = false;
        robot.rrxx__use_relic_grabber = false;
        robot.use_Vuforia = false;
        robot.use_camera = false;

        robot.init(hardwareMap);
        */
        try {
            if (use_wire) {
                mb2 = hardwareMap.get(Wire.class, "mb_ultra");
                mb2.setAddress(0xE0);
            } else if (use_mb) {
                mb_ultra = hardwareMap.get(MB1202.class, "mb_ultra");
            }
        }  catch (Exception e) {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            telemetry.log().add(sw.toString());
            while (true) {
                sleep(1000);
            }
        }
        //mb_ultra.setI2cAddress(MBUltrasonic.ADDRESS_I2C_DEFAULT);
        mr_range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range");
        // get a reference to our digitalTouch object.
        if (use_digit) {
            dSen = hardwareMap.get(DigitalChannel.class, "10cm_prox");
            // set the digital channel to input.
            dSen.setMode(DigitalChannel.Mode.INPUT);
        }
        if (use_wire) {
            mb2.beginWrite(0x51);
            mb2.write(0);
            mb2.endWrite();
        }
        waitForStart();
        long ping_time = System.currentTimeMillis();
        long begin_time = ping_time;
        double dist_cm1 = 0;
        double dist_cm2 = 0;
        int count = 0;
        while (opModeIsActive()) {
            if (use_wire && (System.currentTimeMillis()-ping_time>100)) {
                mb2.requestFrom(0,2);
                mb2.beginWrite(0x51);
                mb2.write(0);
                mb2.endWrite();
                ping_time=System.currentTimeMillis();
            }
            if (mb_ultra!=null) {
                dist_cm1 = mb_ultra.getDistance(DistanceUnit.CM);
                telemetry.addData("1. MB Ultra (I2C) =", "%3.1f cm (Time %3.2f sec)",
                        dist_cm1, (System.currentTimeMillis()-begin_time)/1000.0);
            } else if (mb2!=null) {
                if (mb2.responseCount()>0) {
                    mb2.getResponse();
                    if (mb2.isRead()) {
                        long micro = mb2.micros();
                        dist_cm1 = mb2.readHL();
                        telemetry.addData("1. MB Ultra (Wire) =", "%3.1f cm (Time %3.2f sec)",
                                dist_cm1, (micro/1000.0));
                    }
                }
                telemetry.addData("1. MB Ultra (Wire) =", "%3.1f cm (Time %3.2f sec)",
                        dist_cm1, (System.currentTimeMillis()-begin_time)/1000.0);
            }
            if (mr_range!=null) {
                dist_cm2 = mr_range.getDistance(DistanceUnit.CM);
                telemetry.addData("2. MR Range =", "%3.1f cm (Time %3.2f sec)",
                        dist_cm2, (System.currentTimeMillis()-begin_time)/1000.0);
            }
            if (dSen!=null) {
                dSen_st = dSen.getState();
                telemetry.addData("2. Priximity Sensor =", " %s (Time %3.2f sec)",
                        (dSen_st?"Off":"On"), (System.currentTimeMillis()-begin_time)/1000.0);
            }
            telemetry.update();
            // sleep(40);
        }
        // close all sensors before end the program
        if (mb2!=null) {
            mb2.close();
        }
        if (mb_ultra!=null) {
            mb_ultra.close();
        }
        if (mr_range!=null) {
            mr_range.close();
        }
    }
}
