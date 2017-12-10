package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TuneUp", group="SwerveDrive")
public class TuneUp extends SwerveUtilLOP {

    /* Declare OpMode members. */
    static final double INCREMENT = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.servo_tune_up = true; // enable servo tune up
        robot.use_swerve = false;
        robot.use_imu = false;
        robot.use_Vuforia = false;
        robot.use_color_sensor = false;
        robot.use_arm = true;
        robot.use_glyph_grabber = false;
        robot.use_relic_grabber = true;
        robot.use_test_servo = false;
        robot.use_test_motor = false;
        robot.use_range_sensor = false;

        init_and_test();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("My name is Tobot.", "Need tune-up?");
        telemetry.update();

        int num_servos = 12;
        int cur_sv_ix = 0;
        boolean show_all = true;
        Servo[] sv_list = {
                robot.servoFrontLeft,
                robot.servoFrontRight,
                robot.servoBackLeft,
                robot.servoBackRight,
                robot.sv_shoulder,
                robot.sv_elbow,
                robot.sv_right_arm,
                robot.sv_glyph_grabber_bottom,
                robot.sv_glyph_grabber_top,
                robot.sv_relic_grabber,
                robot.sv_relic_arm
        };
        String [] sv_names = {
                "FrontLeft",
                "FrontRight",
                "BackLeft",
                "BackRight",
                "sv_shoulder",
                "sv_elbow",
                "sv_right_arm",
                "sv_gg_bottom",
                "sv_gg_top",
                "sv_relic_grabber",
                "sv_relic_arm"
        };

        num_servos = sv_list.length;
        while (cur_sv_ix < num_servos && sv_list[cur_sv_ix] == null) {
            cur_sv_ix++;
        }
        if (cur_sv_ix==num_servos) cur_sv_ix = 0; // no servo available

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (robot.use_glyph_grabber) {
            glyph_grabber_auto_open();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad2.dpad_down) {
                double pos = robot.sv_shoulder.getPosition();
                if (pos >= INCREMENT) {
                    robot.sv_shoulder.setPosition(pos - INCREMENT);
                }
            }
            if (gamepad2.dpad_up) {
                double pos = robot.sv_shoulder.getPosition();
                if (pos <= (1 - INCREMENT)) {
                    robot.sv_shoulder.setPosition(pos + INCREMENT);
                }
            }
            if (gamepad2.dpad_right) {
                double pos = robot.sv_elbow.getPosition();
                if (pos <= (1 - INCREMENT)) {
                    robot.sv_elbow.setPosition(pos + INCREMENT);
                }
            }
            if (gamepad2.dpad_left) {
                double pos = robot.sv_elbow.getPosition();
                if (pos >= INCREMENT) {
                    robot.sv_elbow.setPosition(pos - INCREMENT);
                }
            }
            if (gamepad1.dpad_down) {
                double pos = robot.sv_glyph_grabber_bottom.getPosition();
                if (pos >= INCREMENT) {
                    robot.sv_glyph_grabber_bottom.setPosition(pos - INCREMENT);
                }
            }
            if (gamepad1.dpad_up) {
                double pos = robot.sv_glyph_grabber_bottom.getPosition();
                if (pos <= (1 - INCREMENT)) {
                    robot.sv_glyph_grabber_bottom.setPosition(pos + INCREMENT);
                }
            }

            if (gamepad2.a) {
                robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
            }
            if (gamepad2.y) {
                robot.sv_elbow.setPosition(robot.SV_ELBOW_UP);
            }
            if (gamepad2.x) {
                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_LEFT_3);
            }
            if (gamepad2.b) {
                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_RIGHT_3);
            }
            if (gamepad2.left_bumper) {
                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_DOWN);
            }


            if (robot.use_swerve) { // swerve related control

            } // end use_swerve

            if (robot.use_test_motor) {
                if (gamepad1.x) {
                   if (robot.is_gg_upside_down) {
                       test_rotate(-0.4);
                   } else {
                       test_rotate(0.4);
                   }
                }
                if (gamepad1.b) {
                    rotate_refine();
                }
            }
            if (robot.use_glyph_grabber) {
                if (gamepad1.left_bumper) {
                    glyph_grabber_auto_open();
                } else if (gamepad1.left_trigger > 0.1) {
                    // 1. glyph grabber auto close down grabber
                    // 2. if not upside down yet, glyph grabber auto rotates 180 degrees
                    glyph_grabber_auto_close();
                    if (!robot.is_gg_upside_down) {
                        sleep(1000);
                        glyph_grabber_auto_rotate(0.4);
                    }
                }
                if (gamepad1.right_bumper) {
                    glyph_slider_up();
                } else if (gamepad1.right_trigger > 0.1) {
                    glyph_slider_down();
                } else {
                    glyph_slider_stop();
                }
                if (gamepad1.dpad_down) {
                    rotate_refine();
                } else if (gamepad1.dpad_left) {
                    glyph_grabber_all_close();
                    glyph_grabber_auto_rotate(0.4);
                }
            }
            if (robot.servo_tune_up) {
                if (gamepad1.back && gamepad1.a) {
                    show_all = !show_all;
                    sleep(50);
                } else if (gamepad1.a && (sv_list[cur_sv_ix] != null)) {
                    double pos = sv_list[cur_sv_ix].getPosition();
                    if (pos <= (1 - INCREMENT)) {
                        sv_list[cur_sv_ix].setPosition(pos + INCREMENT);
                    }
                    sleep(100);
                } else if (gamepad1.y && (sv_list[cur_sv_ix] != null)) {
                    double pos = sv_list[cur_sv_ix].getPosition();
                    if (pos >= INCREMENT) {
                        sv_list[cur_sv_ix].setPosition(pos - INCREMENT);
                    }
                    sleep(100);
                }
                if (gamepad1.x) {
                    cur_sv_ix--;
                    if (cur_sv_ix < 0) cur_sv_ix = num_servos - 1;
                    int count = 0;
                    while (sv_list[cur_sv_ix] == null && cur_sv_ix >= 0 && count < 20) {
                        cur_sv_ix--;
                        if (cur_sv_ix < 0) cur_sv_ix = num_servos - 1;
                        count++;
                    }
                    gamepad1.reset();
                    sleep(400);
                } else if (gamepad1.b) {
                    cur_sv_ix++;
                    if (cur_sv_ix >= num_servos) cur_sv_ix = 0;
                    int count = 0;
                    while (cur_sv_ix < num_servos && sv_list[cur_sv_ix] == null && count < 20) {
                        cur_sv_ix++;
                        if (cur_sv_ix >= num_servos) cur_sv_ix = 0;
                        count++;
                    }
                    gamepad1.reset();
                    sleep(400);
                }
            }
            telemetry.addData("0. GP1:", "x/b:sv sel, y/a:+/-%4.3f(ix=%d)", INCREMENT, cur_sv_ix);
            if (show_all) {
                for (int i = 0; i < num_servos; i++) {
                    if (sv_list[i] != null) {
                        telemetry.addData("6.", "%d: %s sv-port %d = %5.4f",
                                i, sv_names[i], sv_list[i].getPortNumber(), sv_list[i].getPosition());
                    }
                }
            } else if (sv_list[cur_sv_ix] != null) {
                telemetry.addData("7. Tune-up servo", " %s (ix=%d) = %5.4f",
                        sv_names[cur_sv_ix], cur_sv_ix, sv_list[cur_sv_ix].getPosition());
            } else {
                telemetry.addLine("7. No active servo to tune-up.");
            }
            show_telemetry();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            // robot.waitForTick(40);
        }
    }

}


