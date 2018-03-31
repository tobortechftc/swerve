package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Nick on 9/16/2017.
 */
@TeleOp(name = "Null-Telop", group = "Test")

public class NullTelOp extends SwerveUtilLOP {

    @Override
    public void runOpMode() throws InterruptedException{

        // enable the hardware components to be tested
        robot.use_swerve = false;
        robot.use_minibot = false;
        robot.use_imu = false;
        robot.use_color_sensor = false;
        robot.use_range_sensor = false;
        robot.use_arm = false;
        robot.use_glyph_grabber = false;
        robot.use_relic_grabber = false;
        robot.use_relic_slider = false;
        robot.use_Vuforia = false;
        robot.use_test_servo = true;
        // initialize robot
        robot.init(hardwareMap, telemetry);

        // robot.sv_test.setPosition(0.5);

        waitForStart();
        double val = 0.5;
        while (opModeIsActive()) {
            // It saw a red ball
            if (gamepad2.a) {
                val = 0.0;
            }
            if (gamepad2.y) {
                val = 1.0;
            }
            if (gamepad2.x) {
                val += 0.001;
            }
            if (gamepad2.b) {
                val -= 0.001;
            }
            if (gamepad2.left_bumper) {
                //
            }
            if (val>1.0) val = 1.0;
            else if (val<0.0)
                val = 0.0;
            // robot.sv_test.setPosition(val);

            show_telemetry();
        }

        // stop robot
        stop_tobot();
    }
}