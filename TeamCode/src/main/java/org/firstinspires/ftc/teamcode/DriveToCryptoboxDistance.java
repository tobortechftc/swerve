/*
    Drives forward with range sensor pointing behind it. Waits a short time. Continues until range sensor detects
    enough space between the balance board and itself, then it stops.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Nick on 10/6/2017.
 */

@TeleOp(name = "Swerve GoToCryptoDist", group = "Swerve")
public class DriveToCryptoboxDistance extends SwerveUtilLOP {

    int driveDistance = 40; // 32=close, 49=middle, 70=far
    int mode = 1;
    double speedValue = .5;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.use_Vuforia = false;
        robot.use_imu = false;
        robot.use_encoder = false;
        robot.use_swerve = true;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
        robot.use_color_sensor = false;
        robot.init(hardwareMap);

        telemetry.addData("Say", "Initialization Complete");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean isOverDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) >= driveDistance;
            boolean isUnderDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) <= driveDistance;

            telemetry.addData("Left Power: ", robot.motorFrontLeft.getPower());
            telemetry.addData("Right Power: ", robot.motorFrontRight.getPower());
            telemetry.addData("Distance", robot.rangeSensorBack.getDistance(DistanceUnit.CM));
            telemetry.addData("isOverDistance", isOverDistance);
            telemetry.addData("isUnderDistance,", isUnderDistance);
            telemetry.addData("Mode: ", mode);
            telemetry.addData("Time: ", getRuntime());
            telemetry.update();
            sleep(50);




            if (mode == 1) {    // Drives forward off platform
                driveTT(speedValue, speedValue);
                sleep(1000);
                mode = 2;
            }
            else if (mode == 2) {   // Checks if it's gone far enough
                isOverDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) >= driveDistance;
                if (isOverDistance) {
                    driveTT(-1*speedValue, -1*speedValue);
                    isOverDistance = false;
                    mode = 3;
                }
            }
            else if (mode == 3) { // Drives backwards to compensate
                isUnderDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) <= driveDistance;
                if (isUnderDistance) {
                    driveTT(speedValue/2, speedValue/2);
                    isUnderDistance = false;
                    mode = 4;
                }
            }
            else if (mode == 4) { // Drives forwards to compensate again
                isOverDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) >= driveDistance;
                if (isOverDistance) {
                    driveTT(.0, .0);
                    isOverDistance = false;
                    mode = 5;
                }
            }
            else if (mode == 5) { // End
            }
        }
    }
}