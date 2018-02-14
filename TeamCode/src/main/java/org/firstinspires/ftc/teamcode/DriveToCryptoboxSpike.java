/*
Drive parallel to the wall. Have the range sensor measure each time it increases by a few cm's
indicating it hit one of the dividers (referred to as "spikes") on the cryptobox.

This isn't the final design, just an idea.
 */
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Nick on 9/29/2017.
 */
@TeleOp(name = "Swerve GoToCryptoSpike", group = "Swerve")
@Disabled
public class DriveToCryptoboxSpike extends SwerveUtilLOP {


    final static int SPIKE_DISTANCE = 30;
    final static int SPIKE_PASS_LIMIT = 2; // 1=close, 2=middle, 3=far
    int mode = 1;

    int spikeCount = 0;

    public void detectSpike() {
        if (robot.rangeSensorFrontRight.getDistance(DistanceUnit.CM) <= SPIKE_DISTANCE) {
            spikeCount++;
            sleep(200);
        }
    }
    public boolean isSpike() {
        return robot.rangeSensorFrontRight.getDistance(DistanceUnit.CM) <= SPIKE_DISTANCE;
    }

    @Override
    public void runOpMode() {
        robot.use_Vuforia = false;
        robot.use_imu = false;
        robot.use_encoder = false;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance", robot.rangeSensorFrontRight.getDistance(DistanceUnit.CM));
            telemetry.addData("Spike Count", spikeCount);
            telemetry.addData("Is Spike? ", isSpike());
            telemetry.update();
            sleep(50);
            isSpike();

            // Begins driving forward
            if (mode == 1) {
                driveTT(.4, .4 *.7);
                detectSpike();
                mode = 2;
            }
            else if (mode == 2) {
                detectSpike();
                if (spikeCount >= SPIKE_PASS_LIMIT) {
                    driveTT(-.2, -.2 * .7);
                    mode = 3;
                }
            }
            else if (mode == 3) {
                if (robot.rangeSensorFrontRight.getDistance(DistanceUnit.CM) <= SPIKE_DISTANCE) {
                    sleep(100);
                    driveTT(.0, .0);
                }
            }
        }
    }
}
