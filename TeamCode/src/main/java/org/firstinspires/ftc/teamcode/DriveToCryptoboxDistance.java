/*
    Drives forward with range sensor pointing behind it. Waits a short time. Continues until range sensor detects
    enough space between the balance board and itself, then it stops.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Nick on 10/6/2017.
 */

@Autonomous(name = "RangeSensorTest", group = "Swerve")
@Disabled
public class DriveToCryptoboxDistance extends SwerveUtilLOP {


    class DistanceMetrics implements Func<String> {

        @Override
        public String value() {
            String str = String.format("%f / %f", robot.rangeSensorLeft.getDistance(DistanceUnit.CM), robot.rangeSensorBack.getDistance(DistanceUnit.CM));
            return str;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {


        int mode = 1;
        int driveDistance;
        boolean isOverDistance;
        boolean isUnderDistance;

        double power = -.2;
        int targetColumn = 1;


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


        telemetry.addData("RangeSensorLeft/RangeSensorBack", new DistanceMetrics()).setRetained(true);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Mode", mode);
            telemetry.update();

            switch (targetColumn) { // Defines distance it needs to drive based on column input.
                case 0:
                    driveDistance = 22;
                    break;
                case 1:
                    driveDistance = 38;
                    break;
                case 2:
                    driveDistance = 59;
                    break;
                default:
                    driveDistance = 38;
                    break;
            }

//            if (mode == 1) {
//                driveTT(-.3, -.3);
//                sleep(1250);
//                driveTT(.0, .0);
//                sleep(2000);
//                driveTT(power, power);
//                mode++;
//            } else if (mode == 2) {   // Checks if it's gone far enough
//                isOverDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) >= driveDistance;
//                if (isOverDistance) {
//                    driveTT(-1 * power, -1 * power);
//                    mode++;
//                }
//            } else if (mode == 3) { // Drives backwards to compensate
//                isUnderDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) <= driveDistance;
//                if (isUnderDistance) {
//                    driveTT(power / 2, power / 2);
//                    mode++;
//                }
//            } else if (mode == 4) { // Drives forwards to compensate again (May not need this is robot is precise enough)
//                isOverDistance = robot.rangeSensorBack.getDistance(DistanceUnit.CM) >= driveDistance;
//                if (isOverDistance) {
//                    driveTT(.0, .0);
//                    mode++;
//                }
//            }
        }
    }
}
