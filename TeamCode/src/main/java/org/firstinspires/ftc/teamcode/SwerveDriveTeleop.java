package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Swerve-Teleop", group="SwerveDrive")
public class SwerveDriveTeleop extends SwerveUtilLOP {

    /* Declare OpMode members. */
    //SwerveDriveHardware robot           = new SwerveDriveHardware();

    @Override
    public void runOpMode() throws InterruptedException{
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.use_swerve = true;
        robot.use_imu = true;
        robot.use_encoder = true;
        robot.use_minibot = false;
        robot.use_range_sensor = false;
        robot.use_color_sensor = false;
        robot.use_Vuforia = false;
        robot.use_glyph_grabber = true;
        robot.use_arm = true;

        init_and_test();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        start_init();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // The left stick up/down moves the wheels forward and backwards, while the right stick left/right should rotate the wheels

            if(gamepad1.back && gamepad1.a){
                if(!(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT)){// If in any other mode, switch to snake
                    change_swerve_pos(SwerveDriveHardware.CarMode.STRAIGHT);
                }
                else{ //Return from snake to previous drive mode
                    change_swerve_pos(robot.old_mode);
                }
                sleep(400);
            }
            if(gamepad1.back && gamepad1.start){ // swap test/normalglyph_grabber_auto_open mode
                robot.isTesting = !robot.isTesting;
                sleep(100);
            }

            if (robot.use_swerve) {

                if(robot.isTesting){ //Allow to test individual servo positions

                    if (gamepad1.left_trigger > 0.1) {

                        sleep(100);
                        TurnLeftD(0.5, 90.0);
                    }
                    if (gamepad1.right_trigger > 0.1) {
                        sleep(100);
                        TurnRightD(0.5, 90.0);
                    }

                    if (gamepad1.b) {
                        StraightIn(0.5, 22);
                    }

                    if (gamepad1.a){
                        StraightIn(-0.5, 22);
                    }

                    if(gamepad1.start){
                        if(!(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB)){// If in any other mode, switch to crab
                            change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
                        }
                        else{ //Return from snake to previous drive mode
                            change_swerve_pos(robot.old_mode);
                        }
                        sleep(400);
                    }
                }
                else { //If not allowed to test servo positions, triggers do teleop spot turn
                    //if (gamepad1.left_trigger > 0.1) {
                    if (gamepad1.right_stick_x < -0.1) {
                        change_swerve_pos(SwerveDriveHardware.CarMode.TURN);

                        sleep(200);

                        // while (gamepad1.left_trigger > 0.1) {
                        while (gamepad1.right_stick_x < -0.1) {
                            robot.motorFrontLeft.setPower(robot.drivePowerRatio);
                            robot.motorFrontRight.setPower(-1*robot.drivePowerRatio);
                            robot.motorBackLeft.setPower(robot.drivePowerRatio);
                            robot.motorBackRight.setPower(-1*robot.drivePowerRatio);
                        }

                        change_swerve_pos(robot.old_mode);

                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }
                    //if (gamepad1.right_trigger > 0.1) {
                    if (gamepad1.right_stick_x > 0.1) {
                        change_swerve_pos(SwerveDriveHardware.CarMode.TURN);

                        sleep(200);

                        //while (gamepad1.right_trigger > 0.1) {
                        while (gamepad1.right_stick_x > 0.1) {
                            robot.motorFrontLeft.setPower(-1*robot.drivePowerRatio);
                            robot.motorFrontRight.setPower(robot.drivePowerRatio);
                            robot.motorBackLeft.setPower(-1*robot.drivePowerRatio);
                            robot.motorBackRight.setPower(robot.drivePowerRatio);
                        }

                        change_swerve_pos(robot.old_mode);

                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }
                    if (gamepad1.left_bumper) {
                        change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);

                        sleep(200);

                        while (gamepad1.left_bumper) {
                            robot.motorFrontLeft.setPower(-0.3);
                            robot.motorFrontRight.setPower(0.3);
                            robot.motorBackLeft.setPower(0.3);
                            robot.motorBackRight.setPower(-0.3);
                        }

                        change_swerve_pos(robot.old_mode);

                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }

                    if (gamepad1.right_bumper) {
                        change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);

                        sleep(200);

                        while (gamepad1.right_bumper) {
                            robot.motorFrontLeft.setPower(0.3);
                            robot.motorFrontRight.setPower(-0.3);
                            robot.motorBackLeft.setPower(-0.3);
                            robot.motorBackRight.setPower(0.3);
                        }

                        change_swerve_pos(robot.old_mode);

                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }
                    if (gamepad1.a && gamepad1.y) {
                        robot.drivePowerRatio = 0.7;
                        sleep(20);
                    } else if (gamepad1.y) { // slow mode
                        robot.drivePowerRatio = 0.5;
                    } else if (gamepad1.a) {
                        robot.drivePowerRatio = 0.2;
                    }
                    else if (gamepad1.dpad_down){
                        robot.drivePowerRatio -=0.05;
                        if(robot.drivePowerRatio < 0.1){
                            robot.drivePowerRatio = 0.1;
                        }
                    }
                    else if (gamepad1.dpad_up){
                        robot.drivePowerRatio += 0.05;
                        if(robot.drivePowerRatio > 1.0){
                            robot.drivePowerRatio = 1.0;
                        }
                    }

                }




                if (robot.cur_mode == SwerveDriveHardware.CarMode.CAR) { //If in snake drive, calculate and change servo angles

                    //calc_snake(gamepad1.right_stick_x);
                    calc_snake(gamepad1.left_trigger, gamepad1.right_trigger);
                    snake_servo_adj();
                }
                else { //


                    if (gamepad1.x) { //Cycle through non-snake drive modes
                        if (robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT) {

                            change_swerve_pos(SwerveDriveHardware.CarMode.TURN);

                        } else if (robot.cur_mode == SwerveDriveHardware.CarMode.TURN) {

                            change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);

                        } else {

                            change_swerve_pos(SwerveDriveHardware.CarMode.STRAIGHT);

                        }
                        sleep(400);
                    }

                }

                // Possible idea where holding the left stick farther from the center makes it turn the servo farther. Not completed.
//            while (-gamepad1.left_stick_y > .10 && -gamepad1.left_stick_y < -.10) {
//                servoPos = -gamepad1.left_stick_y * .2;
//            }

                correct_swerve_servos();

                //set_swerve_power(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);
                set_swerve_power(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);

            } // end use_swerve

            if (robot.use_relic_grabber) {
                // relic slider
               if (gamepad2.left_stick_y>0.1) {
                   robot.mt_relic_slider.setPower(-0.5);
               } else if (gamepad2.left_stick_y<-0.1) {
                    robot.mt_relic_slider.setPower(0.5);
               } else {
                    robot.mt_relic_slider.setPower(0);
               }

                // relic arm
                if (gamepad2.right_stick_y>0.1) {
                    double cur_pos = robot.sv_relic_arm.getPosition();
                    if (cur_pos<0.95) {
                        robot.sv_relic_arm.setPosition(cur_pos + 0.01);
                    }
                } else if (gamepad2.right_stick_y<-0.1) {
                    double cur_pos = robot.sv_relic_arm.getPosition();
                    if (cur_pos>0.05) {
                        robot.sv_relic_arm.setPosition(cur_pos - 0.01);
                    }
                }

                // relic grabber open/close
                if (gamepad2.x && gamepad2.back) {
                    relic_grabber_close();
                } else if (gamepad2.x) {
                    relic_grabber_open();
                }
            }

            if (robot.use_glyph_grabber) {
                if (gamepad2.back && gamepad2.b) { //set glyph grabber to original open position
                    glyph_grabber_auto_init();
                    robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_OPEN);
                    robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_OPEN);
                } else if (gamepad2.right_bumper && gamepad2.left_bumper) {
                    glyph_grabber_auto_init();
                } else if (gamepad2.left_bumper && gamepad2.b) {
                    glyph_grabber_open_and_push();
                } else if (gamepad2.left_bumper && gamepad2.y) {
                    glyph_grabber_open_top();
                } else if (gamepad2.left_bumper && gamepad2.a) {
                    glyph_grabber_open_bottom();
                } else if (gamepad2.left_bumper) {
                    glyph_grabber_auto_open();
                    // should slide back to init
                    // glyph_slider_init();
                } else if (gamepad2.back && (gamepad2.left_trigger > 0.1)) { // half close
                    glyph_grabber_half_close();
                } else if (gamepad2.dpad_up && (gamepad2.left_trigger > 0.1)) { // half close
                    glyph_grabber_half_close_both();
                } else if (gamepad2.b && (gamepad2.left_trigger > 0.1)) { // close both
                    glyph_grabber_all_close();
                } else if (gamepad2.a && (gamepad2.left_trigger > 0.1)) { // close one + rotate
                    // 1. glyph grabber auto close down grabber
                    // 2. if not upside down yet, glyph grabber auto rotates 180 degrees
                    glyph_grabber_auto_close();
                    if (!robot.is_gg_upside_down) {
                        sleep(1000);
                        glyph_grabber_auto_rotate(1.0);
                    }
                } else if (gamepad2.left_trigger > 0.1) {
                    glyph_grabber_auto_close();
                } else if (gamepad2.a && gamepad2.dpad_down) {
                    glyph_slider_init();
                } else if (gamepad2.dpad_down) {
                    glyph_slider_down_auto();
                } else if (gamepad2.dpad_up) {
                    glyph_slider_up_auto();
                } else if (gamepad2.dpad_left) {
                    glyph_grabber_auto_rotate(0.7);
                } else if (gamepad2.dpad_right) {
                    rotate_refine();
                    // glyph_grabber_auto_rotate(0.7);
                } else if (gamepad2.right_bumper) { // manual up
                    glyph_slider_up();
                } else if (gamepad2.back && gamepad2.right_trigger > 0.1) { // down and reset
                    glyph_slider_down_and_reset();
                }else if (gamepad2.right_trigger > 0.1) { // manual down
                    glyph_slider_down();
                } else {
                    glyph_slider_stop();
                }

            }
            show_telemetry();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
