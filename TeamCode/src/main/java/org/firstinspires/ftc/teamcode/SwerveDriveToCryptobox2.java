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
public class SwerveDriveToCryptobox2 extends LinearOpMode {
    SwerveDriveHardware hw = new SwerveDriveHardware();

    int driveDistance = 120; // 120=close, 139=middle, 158=far


    @Override
    public void runOpMode() {
        hw.use_Vuforia = false;
        hw.use_imu = false;
        hw.use_encoder = false;
        hw.use_minibot = true;
        hw.use_range_sensor = true;
        hw.init(hardwareMap);

        while (opModeIsActive()) {
            telemetry.addData("Distance: ", hw.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            sleep(100);

            hw.motorFrontLeft.setPower(.3);
            hw.motorFrontRight.setPower(.3);

            if (hw.rangeSensor.getDistance(DistanceUnit.CM) >= driveDistance) {
                hw.motorFrontLeft.setPower(0.0);
                hw.motorFrontRight.setPower(0.0);
            }
        }
    }
}
