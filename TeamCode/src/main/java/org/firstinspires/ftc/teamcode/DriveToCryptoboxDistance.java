/*
    Drives forward with range sensor pointing behind it. Waits a short time. Continues until range sensor detects
    enough space between the balance board and itself, then it stops.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Nick on 10/6/2017.
 */

@Autonomous(name = "Drive-Range", group = "Swerve")
public class DriveToCryptoboxDistance extends SwerveUtilLOP {

    int driveDistance = 49; // 32=close, 49=middle, 70=far
    int mode = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.use_Vuforia = false;
        robot.use_imu = false;
        robot.use_encoder = false;
        robot.use_minibot = true;
        robot.use_range_sensor = true;
        robot.use_color_sensor = false;
        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            driveTT(0.3,0.3);
            sleep(2000);
            driveTT(0,0);
            requestOpModeStop();
        }
        stop_auto();

        //while (opModeIsActive()) {
        //    boolean isDistance = robot.rangeSensor.getDistance(DistanceUnit.CM) >= driveDistance &&
          //                       robot.rangeSensor.getDistance(DistanceUnit.CM) >= driveDistance;
         //   telemetry.addData("Left Power: ", robot.motorFrontLeft.getPower());
         //   telemetry.addData("Right Power: ", robot.motorFrontRight.getPower());
         //   telemetry.addData("Distance", robot.rangeSensor.getDistance(DistanceUnit.CM));
         //   telemetry.addData("isDistance", isDistance);
         //   telemetry.addData("Mode: ", mode);
         //   telemetry.addData("Time: ", getRuntime());
         //   telemetry.update();
         //   sleep(50);




//            if (mode == 1) {
//                driveTT(.2, .25);
//                sleep(500);
//                mode = 2;
//            }
//            else if (mode == 2) {
//                isDistance = robot.rangeSensor.getDistance(DistanceUnit.CM) >= driveDistance &&
//                             robot.rangeSensor.getDistance(DistanceUnit.CM) >= driveDistance;
//                if (isDistance) {
//                    driveTT(0, 0);
//                    mode = 3;
//                }
//            }
//            else if (mode == 3) {
//                // end
//            }
//        }
    }
}