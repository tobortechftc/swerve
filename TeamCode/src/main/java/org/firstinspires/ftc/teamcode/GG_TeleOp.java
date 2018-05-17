package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="GG-TeleOp", group="SwerveDrive")
@Disabled
public class GG_TeleOp extends SwerveUtilLOP {

    /* Declare OpMode members. */
    static final double INCREMENT = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
//        /* Initialize the hardware variables.
//         * The init() method of the hardware class does all the work here
//         */
//        boolean serve_tune_up = false; // enable servo tune up
//        robot.use_swerve = false;
//        robot.use_imu = false;
//        robot.use_Vuforia = false;
//        robot.use_color_sensor = false;
//        robot.use_arm = false;
//        robot.use_glyph_grabber = true;
//        robot.use_test_motor = false;
//
//        init_and_test();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            if (robot.use_swerve) { // swerve related control
//
//            } // end use_swerve
//
//            if (robot.use_glyph_grabber) {
//                if (gamepad2.left_bumper) {
//                    glyph_grabber_auto_open();
//                    // should slide back to init
//                    // glyph_slider_init();
//                } else if (gamepad2.b && (gamepad2.left_trigger > 0.1)) { // close both
//                    glyph_grabber_all_close();
//                } else if (gamepad2.a && (gamepad2.left_trigger > 0.1)) { // close one + rotate
//                    // 1. glyph grabber auto close down grabber
//                    // 2. if not upside down yet, glyph grabber auto rotates 180 degrees
//                    glyph_grabber_auto_close(false,false);
//                    if (!robot.is_gg_upside_down) {
//                        sleep(1000);
//                        glyph_grabber_auto_rotate(0.4);
//                    }
//                } else if (gamepad2.left_trigger > 0.1) {
//                    glyph_grabber_auto_close(false,false);
//                }
//                if (gamepad2.a && gamepad2.dpad_down) {
//                    glyph_slider_init();
//                } else if (gamepad2.dpad_down) {
//                    glyph_slider_down_auto();
//                } else if (gamepad2.dpad_up) {
//                    glyph_slider_up_auto();
//                } else if (gamepad2.dpad_left) {
//                    glyph_grabber_auto_rotate(0.4);
//                } else if (gamepad2.dpad_right) {
//                    // glyph_grabber_auto_rotate(0.4);
//                } else if (gamepad2.right_bumper) { // manual up
//                    glyph_slider_up();
//                } else if (gamepad2.right_trigger > 0.1) { // manual down
//                    glyph_slider_down();
//                } else {
//                    glyph_slider_stop();
//                }
//            }
//
//            show_telemetry();
//
//            // robot.waitForTick(40);
//        }
    }

}


