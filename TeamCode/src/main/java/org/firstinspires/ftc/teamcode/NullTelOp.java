package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Nick on 9/16/2017.
 */
@TeleOp(name = "Null-Telop", group = "Test")
@Disabled
public class NullTelOp extends SwerveUtilLOP {

    @Override
    public void runOpMode() throws InterruptedException{

        // enable the hardware components to be tested
        robot.use_swerve = false;
        robot.use_imu = false;
        robot.use_color_sensor = false;
        robot.use_range_sensor = false;
        robot.use_arm = false;
        robot.use_glyph_grabber = false;
        robot.use_relic_grabber = false;
        robot.use_Vuforia = false;

        // initialize robot
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // It saw a red ball
            if (gamepad2.a) {
                //
            }
            if (gamepad2.y) {
                //
            }
            if (gamepad2.x) {
                //
            }
            if (gamepad2.b) {
                //
            }
            if (gamepad2.left_bumper) {
                //
            }

            show_telemetry();
        }

        // stop robot
        stop_tobot();
    }
}