/*
    Drives forward with range sensor pointing behind it. Waits a short time. Continues until range sensor detects
    enough space between the balance board and itself, then it stops.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Nick on 10/6/2017.
 */

@Autonomous(name = "RangeSensorTest", group = "Swerve")
public class DriveToCryptoboxDistance extends SwerveUtilLOP {


    @Override
    public void runOpMode() throws InterruptedException {


        int mode = 1;
        int driveDistance;
        boolean isOverDistance;
        boolean isUnderDistance;
        telemetry.addData("Mode", mode);

        double power = .5;
        int targetColumn = 2;


        robot.use_swerve = true;
        robot.use_imu = false;
        robot.use_encoder = true;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
        robot.use_color_sensor = false;
        robot.use_Vuforia = false;

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Mode", mode);
            telemetry.addData("RangeSensorLeft", robot.rangeSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("RangeSensorBack", robot.rangeSensorBack.getDistance(DistanceUnit.CM));
            sleep(50);
            telemetry.update();

            switch (targetColumn) { // Defines distance it needs to drive based on column input.
                case 1:
                    driveDistance = 22;
                    break;
                case 2:
                    driveDistance = 38;
                    break;
                case 3:
                    driveDistance = 59;
                    break;
                default:
                    driveDistance = 38;
                    break;
            }

            if (mode == 1) {    // Drives forward
                driveTT(power, power);
                mode = 2;
            } else if (mode == 2) {   // Checks if it's gone far enough
                isOverDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) >= driveDistance;
                if (isOverDistance) {
                    driveTT(-1 * power, -1 * power);
                    mode = 3;
                }
            } else if (mode == 3) { // Drives backwards to compensate
                isUnderDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) <= driveDistance;
                if (isUnderDistance) {
                    driveTT(power * 3 / 4, power * 3 / 4);
                    mode = 4;
                }
            } else if (mode == 4) { // Drives forwards to compensate again (May not need this is robot is precise enough)
                isOverDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) >= driveDistance;
                if (isOverDistance) {
                    driveTT(.0, .0);
                    mode = 5;
                }
            }
        }
    }
}