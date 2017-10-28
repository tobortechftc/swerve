/*
Drive parallel to the wall. Have the range sensor measure each time it increases by a few cm's
indicating it hit one of the dividers (referred to as "spikes") on the cryptobox.

This isn't the final design, just an idea.
 */
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Nick on 9/29/2017.
 */
@TeleOp(name = "Swerve: DriveToCryptoboxSpike", group = "Swerve")
public class DriveToCryptoboxSpike extends SwerveUtilLOP {


    final static int SPIKE_DISTANCE = 40;
    final static int SPIKE_PASS_LIMIT = 2; // 1=close, 2=middle, 3=far

    int spikeCount = 0;

    public void detectSpike() {
        if (robot.rangeSensor.getDistance(DistanceUnit.CM) <= SPIKE_DISTANCE) {
            spikeCount++;
            sleep(250);
        }
    }

    @Override
    public void runOpMode() {
        robot.use_Vuforia = false;
        robot.use_imu = false;
        robot.use_encoder = false;
        robot.use_minibot = true;
        robot.use_range_sensor = true;
        robot.use_color_sensor = false;
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance: ", robot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Spike Count: ", spikeCount);
            telemetry.update();
            sleep(50);

            // Hasn't detected enough spikes
            if (spikeCount < SPIKE_PASS_LIMIT) {
                driveTT(.5, .5 *.7);
                detectSpike();
            }
            // It has reached the correct number of spikes
            else {
                driveTT(.0, .0);
            }
        }
    }
}
