/*
    Drives forward with range sensor pointing behind it. Continues until range sensor detects
    enough space, then it stops.
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

@TeleOp(name = "Swerve: DriveToCryptobox2", group = "Swerve")
public class SwerveDriveToCryptobox2 extends SwerveUtilLOP {

    int driveDistance = 120; // 120=close, 139=middle, 158=far


    @Override
    public void runOpMode() {
        robot.use_Vuforia = false;
        robot.use_imu = false;
        robot.use_encoder = false;
        robot.use_minibot = true;
        robot.use_range_sensor = true;
        robot.use_color_sensor = true;
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            boolean isDistance = robot.rangeSensor.getDistance(DistanceUnit.CM) <= driveDistance;

            telemetry.addData("Distance: ", robot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("isDistance: ", isDistance);
            telemetry.update();
            sleep(50);
        }
    }
}