package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="TeleOp-H", group="SwerveDrive")
public class TeleopH extends SwerveUtilLOP {

    /* Declare OpMode members. */
    //SwerveDriveHardware robot           = new SwerveDriveHardware();

    @Override
    public void runOpMode() throws InterruptedException{
//        int adj_count = 0; // count to smooth out car turn
//
//        /* Initialize the hardware variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.use_swerve = true;
//        robot.use_newbot = false;
//        robot.use_front_drive_only = false; // front drive only
//        robot.use_intake = false;
//        robot.use_imu = true;
//        robot.use_encoder = true;
//        robot.use_minibot = false;
//        robot.use_range_sensor = false;
//        robot.use_color_sensor = false;
//        robot.use_Vuforia = false;
//        robot.use_glyph_grabber = false;
//        robot.use_relic_grabber = false;
//        robot.use_relic_slider = false;
//        robot.use_arm = false;
//
//        init_and_test();
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Say", "Initialization Complete");    //
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        start_init();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
//            // The left stick up/down moves the wheels forward and backwards, while the right stick left/right should rotate the wheels
//
//            if(gamepad1.back && gamepad1.start){ // swap test/normalglyph_grabber_auto_open mode
//                robot.isTesting = !robot.isTesting;
//                sleep(100);
//            }
//            if (robot.use_intake) {
//                if (gamepad1.dpad_up) { // intake IN
//                    intakeIn();
//                } else if (gamepad1.dpad_down) { // intake IN
//                    intakeOut();
//                } else if (gamepad1.dpad_left) {
//                    intakeStop();
//                }
//
//            }
//
//            if (robot.use_swerve) {
//
//                if(robot.isTesting){ //Allow to test individual movement
//
//                    if (gamepad1.dpad_left) {
//                        sleep(100);
//                        TurnLeftD(0.5, 90.0);
//                    }
//                    if (gamepad1.dpad_right) {
//                        sleep(100);
//                        TurnRightD(0.5, 90.0);
//                    }
//
//                    if(gamepad1.left_trigger > 0.1){
//                        robot.NB_LEFT_SV_DIFF -= 0.001;
//                        sleep(100);
//                    }
//
//                    if(gamepad1.left_bumper){
//                        robot.NB_LEFT_SV_DIFF += 0.001;
//                        sleep(100);
//                    }
//
//                    if(gamepad1.right_trigger > 0.1){
//                        robot.NB_RIGHT_SV_DIFF -= 0.001;
//                        sleep(100);
//                    }
//
//                    if(gamepad1.right_bumper){
//                        robot.NB_RIGHT_SV_DIFF += 0.001;
//                        sleep(100);
//                    }
//
//                    if (gamepad1.b) {
//                        StraightIn(0.5, 22);
//                    }
//
//                    if (gamepad1.a){
//                        StraightIn(-0.5, 22);
//                    }
//
//                    if(gamepad1.start){
//                        if(!(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB)){// If in any other mode, switch to crab
//                            change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
//                        }
//                        else{ //Return from snake to previous drive mode
//                            change_swerve_pos(robot.old_mode);
//                        }
//                        sleep(200);
//                    }
//                    if (robot.use_newbot) {
//                        robot.initialize_newbot();
//                    }
//                } // ens isTesting
//                else { //If not allowed to test servo positions, triggers do teleop spot turn
//                    //if (gamepad1.left_trigger > 0.1) {
//                    if (Math.abs(gamepad1.left_stick_y) > 0.1 &&
//                            Math.abs(gamepad1.right_stick_y) > 0.1 &&
//                            Math.abs(gamepad1.left_stick_y - gamepad1.right_stick_y) > 0.2 ) {
//                        // Swerve Turn
//                        if(robot.cur_mode != SwerveDriveHardware.CarMode.TURN) {
//                            change_swerve_pos(SwerveDriveHardware.CarMode.TURN);
//                            // sleep(200);
//                        }
//                        double pw_dir = (gamepad1.right_stick_y>0.1?-1.0:1.0);
//                        while (Math.abs(gamepad1.left_stick_y -gamepad1.right_stick_y) > 0.1) {
//                            robot.motorFrontLeft.setPower(robot.drivePowerRatio*pw_dir);
//                            robot.motorFrontRight.setPower(-1*robot.drivePowerRatio*pw_dir);
//                            robot.motorBackLeft.setPower(robot.drivePowerRatio*pw_dir);
//                            robot.motorBackRight.setPower(-1*robot.drivePowerRatio*pw_dir);
//                        }
//                        robot.motorFrontLeft.setPower(0);
//                        robot.motorFrontRight.setPower(0);
//                        robot.motorBackLeft.setPower(0);
//                        robot.motorBackRight.setPower(0);
//                    } else if (Math.abs(gamepad1.left_stick_x)>0.2 &&
//                            Math.abs(gamepad1.right_stick_x)>0.2 &&
//                            Math.abs(gamepad1.left_stick_x+gamepad1.right_stick_x)>0.4) {
//                        // crab move left / right
//                        if (robot.cur_mode != SwerveDriveHardware.CarMode.CRAB) {
//                            change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
//                            sleep(200);
//                        }
//                        double pw_dir = (gamepad1.left_stick_x>0.1?-1.0:1.0);
//                        while (Math.abs(gamepad1.left_stick_x+gamepad1.right_stick_x)>0.2) {
//                            robot.motorFrontLeft.setPower(-robot.drivePowerRatio*pw_dir);
//                            robot.motorFrontRight.setPower(robot.drivePowerRatio*pw_dir);
//                            robot.motorBackLeft.setPower(-robot.drivePowerRatio*pw_dir);
//                            robot.motorBackRight.setPower(robot.drivePowerRatio*pw_dir);
//                        }
//                        robot.motorFrontLeft.setPower(0);
//                        robot.motorFrontRight.setPower(0);
//                        robot.motorBackLeft.setPower(0);
//                        robot.motorBackRight.setPower(0);
//                    } else if (Math.abs(gamepad1.left_stick_y)>0.1) {
//                        if (robot.cur_mode != SwerveDriveHardware.CarMode.CAR) {
//                            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
//                            sleep(200);
//                        }
//                    }
//                    else if (gamepad1.x){
//                        if (robot.cur_mode != SwerveDriveHardware.CarMode.ORBIT) {
//                            change_swerve_pos(SwerveDriveHardware.CarMode.ORBIT);
//                            sleep(200);
//                        }
//                        while (gamepad1.x) {
//                            robot.motorFrontLeft.setPower(-robot.drivePowerRatio);
//                            robot.motorFrontRight.setPower(robot.drivePowerRatio);
//                            robot.motorBackLeft.setPower(-robot.drivePowerRatio);
//                            robot.motorBackRight.setPower(robot.drivePowerRatio);
//                        }
//                        // change_swerve_pos(robot.old_mode);
//                        robot.motorFrontLeft.setPower(0);
//                        robot.motorFrontRight.setPower(0);
//                        robot.motorBackLeft.setPower(0);
//                        robot.motorBackRight.setPower(0);
//                    } else if (gamepad1.b){
//                        if (robot.cur_mode != SwerveDriveHardware.CarMode.ORBIT) {
//                            change_swerve_pos(SwerveDriveHardware.CarMode.ORBIT);
//                            sleep(200);
//                        }
//                        while (gamepad1.b) {
//                            robot.motorFrontLeft.setPower(robot.drivePowerRatio);
//                            robot.motorFrontRight.setPower(-robot.drivePowerRatio);
//                            robot.motorBackLeft.setPower(robot.drivePowerRatio);
//                            robot.motorBackRight.setPower(-robot.drivePowerRatio);
//                        }
//                        // change_swerve_pos(robot.old_mode);
//                        robot.motorFrontLeft.setPower(0);
//                        robot.motorFrontRight.setPower(0);
//                        robot.motorBackLeft.setPower(0);
//                        robot.motorBackRight.setPower(0);
//                    }
//
//                    if (gamepad1.a && gamepad1.y) {
//                        robot.drivePowerRatio = 0.7; // turbo mode
//                        sleep(20);
//                    } else if (gamepad1.y) { // fast mode
//                        robot.drivePowerRatio = 0.5;
//                    } else if (gamepad1.a) { // slow mode
//                        robot.drivePowerRatio = 0.2;
//                    }
//                    /*
//                    else if (gamepad1.dpad_down){
//                        robot.drivePowerRatio -=0.01;
//                        if(robot.drivePowerRatio < 0.1){
//                            robot.drivePowerRatio = 0.1;
//                        }
//                    }
//                    else if (gamepad1.dpad_up){
//                        robot.drivePowerRatio += 0.01;
//                        if(robot.drivePowerRatio > 1.0){
//                            robot.drivePowerRatio = 1.0;
//                        }
//                    }
//                    */
//                }
//
//                if (robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
//                    // right_stick_x to control snake
//                    if (Math.abs(gamepad1.left_stick_y)>0.1 && Math.abs(gamepad1.right_stick_x)>0.2) {
//                        calc_snake(-gamepad1.right_stick_x, gamepad1.right_stick_x);
//                    } else {
//                        calc_snake(0, 0);
//                    }
//                    snake_servo_adj();
//                }
//
//                correct_swerve_servos();
//
//                //set_swerve_power(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);
//                set_swerve_power(gamepad1.left_stick_y, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
//
//                if (robot.cur_mode==SwerveDriveHardware.CarMode.CAR ||
//                    robot.cur_mode==SwerveDriveHardware.CarMode.STRAIGHT) {
//                    // use gamepad1.joy_stick_x to turn like Car
//                    if ((adj_count++)%20==0)
//                       car_servo_adj(gamepad1.left_stick_x);
//                }
//            } // end use_swerve
//
//            if (robot.use_relic_slider) {
//                // relic slider
//                if (gamepad2.y && gamepad2.back) {
//                    relic_slider_out_max();
//                } else if (gamepad2.a & gamepad2.back) {
//                    relic_slider_back_auto();
//                } else if (gamepad2.left_stick_y > 0.1) {
//                    robot.mt_relic_slider.setPower(1.0);
//                } else if (gamepad2.left_stick_y < -0.1) {
//                    robot.mt_relic_slider.setPower(-1.0);
//                } else {
//                    robot.mt_relic_slider.setPower(0);
//                }
//            }
//
//            if (robot.use_glyph_grabber) {
//                if ((gamepad2.right_stick_x<-0.1) && (gamepad2.left_bumper)) {
//                    // top grabber inc. widen
//                    glyph_grabber_top_widen();
//                } else if ((gamepad2.right_stick_x>0.1) && (gamepad2.left_bumper)) {
//                    // bottom grabber inc. close
//                    glyph_grabber_top_closer();
//                } else if (gamepad2.back && gamepad2.b) { //set glyph grabber to original open position
//                    glyph_grabber_auto_init();
//                    robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_OPEN);
//                    robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_OPEN);
//                } else if (gamepad2.right_bumper && gamepad2.left_bumper) {
//                    glyph_grabber_auto_init();
//                } else if (gamepad2.left_bumper && gamepad2.y) {
//                    glyph_grabber_open_top();
//                } else if (gamepad2.left_bumper && gamepad2.a) {
//                    glyph_grabber_open_bottom();
//                } else if ((gamepad2.left_bumper&&!gamepad2.y&&!gamepad2.a) || gamepad1.dpad_left) {
//                    glyph_grabber_auto_open();
//                    // should slide back to init
//                    // glyph_slider_init();
//                } else if (gamepad2.back && (gamepad2.left_trigger > 0.1)) { // half close
//                    glyph_grabber_half_close();
//                    sleep(400);
//                } else if (gamepad2.dpad_up && (gamepad2.left_trigger > 0.1)) { // close top
//                    glyph_grabber_auto_close(true,false);
//                    sleep(400);
//                } else if (gamepad2.dpad_down && (gamepad2.left_trigger > 0.1)) { // half close
//                    glyph_grabber_half_close_both();
//                    sleep(400);
//                } else if ((gamepad2.b && (gamepad2.left_trigger > 0.1)) ||
//                        (gamepad1.back && gamepad1.dpad_right)) { // close both
//                    glyph_grabber_all_close();
//                    sleep(400);
//                } else if (gamepad2.right_stick_x>0.1 && (gamepad2.left_trigger > 0.1)) {
//                    // bottom grabber inc. close
//                    glyph_grabber_bottom_closer();
//                } else if (gamepad2.right_stick_x<-0.1 && (gamepad2.left_trigger > 0.1)) {
//                    // bottom grabber inc. widen
//                    glyph_grabber_bottom_widen();
//                } else if ((gamepad2.left_trigger > 0.1) || gamepad1.dpad_right) {
//                    glyph_grabber_auto_close(false,false);
//                } else if (gamepad2.a && gamepad2.dpad_down) {
//                    glyph_slider_init();
//                } else if (gamepad2.back && gamepad2.right_bumper) { // force up
//                    glyph_slider_up_force();
//                } else if (gamepad2.dpad_down) {
//                    glyph_slider_down_auto();
//                } else if (gamepad2.dpad_up) {
//                    glyph_slider_up_auto();
//                } else if (gamepad2.dpad_left) {
//                    glyph_grabber_auto_rotate(1.0);
//                } else if (gamepad2.dpad_right && gamepad2.b) {
//                    rotate_refine_up();
//                } else if (gamepad2.dpad_right && gamepad2.x) {
//                    rotate_refine_down();
//                } else if (gamepad2.dpad_right) {
//                    rotate_refine();
//                } else if (gamepad2.right_bumper || gamepad1.dpad_up) { // manual up
//                    glyph_slider_up();
//                } else if (gamepad2.back && gamepad2.right_trigger > 0.1) { // down and reset
//                    glyph_slider_down_and_reset();
//                }else if ((gamepad2.right_trigger > 0.1) || gamepad1.dpad_down) { // manual down
//                    glyph_slider_down();
//                } else {
//                    glyph_slider_stop();
//                }
//
//            }
//            if (robot.use_relic_grabber) {
//                // relic arm
//                if (gamepad2.right_stick_y<-0.1) {
//                    double cur_pos = robot.sv_relic_wrist.getPosition();
//                    if (cur_pos<0.99) {
//                        robot.sv_relic_wrist.setPosition(cur_pos + 0.005);
//                    }
//                } else if (gamepad2.right_stick_y>0.1) {
//                    double cur_pos = robot.sv_relic_wrist.getPosition();
//                    if (cur_pos>0.01) {
//                        robot.sv_relic_wrist.setPosition(cur_pos - 0.005);
//                    }
//                } else if (gamepad2.a && gamepad2.y) {
//                    relic_arm_middle();
//                } else if (gamepad2.y && !gamepad2.left_bumper) {
//                    relic_arm_up();
//                } else if (gamepad2.a && !gamepad2.left_bumper) {
//                    relic_arm_down();
//                }
//                // relic grabber open/close
//                if (gamepad2.b && !gamepad2.start) {
//                    relic_grabber_close();
//                } else if (gamepad2.x && gamepad2.back) {
//                    // relic_grabber_release();
//                    auto_relic_release();
//                }  else if (gamepad2.x && !gamepad2.dpad_right) {
//                    relic_grabber_release();
//                    // auto_relic_release();
//                } else if (gamepad2.a && gamepad2.y) {
//                    relic_arm_middle();
//                } else if (gamepad2.y) {
//                    relic_arm_up();
//                } else if (gamepad2.a) {
//                    relic_arm_down();
//                }
//            }
//            show_telemetry();
//
//            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//            // robot.waitForTick(40);
//        }
    }
}
