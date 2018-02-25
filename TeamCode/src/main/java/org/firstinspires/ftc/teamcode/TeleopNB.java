package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TeleOp-NB", group="Test-NewBot")
public class TeleopNB extends SwerveUtilLOP {

    /* Declare OpMode members. */
    //SwerveDriveHardware robot           = new SwerveDriveHardware();

    @Override
    public void runOpMode() throws InterruptedException{
        int adj_count = 0; // count to smooth out car turn

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.use_swerve = false;
        robot.use_newbot = true;
        robot.use_front_drive_only = false; // front drive only
        robot.use_intake = true;
        robot.use_dumper = true;
        robot.use_imu = false;
        robot.use_encoder = true;
        robot.use_minibot = false;
        robot.use_range_sensor = false;
        robot.use_color_sensor = false;
        robot.use_proximity_sensor = false;
        robot.use_Vuforia = false;
        robot.use_glyph_grabber = false;
        robot.use_relic_grabber = true;
        robot.use_relic_slider = true;
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

            if(gamepad1.back && gamepad1.start){ // swap test/normalglyph_grabber_auto_open mode
                robot.isTesting = !robot.isTesting;
                sleep(100);
            }
            if (robot.use_intake && !robot.isTesting) {
                if (gamepad1.x || (gamepad2.left_bumper&&!gamepad2.right_bumper)) { // intake IN
                    intakeIn();
                } else if (gamepad1.b || (gamepad2.left_trigger > 0.1)) { // intake OUT
                    intakeOut();
                } else {
                    intakeStop();
                }
                if (gamepad1.dpad_up && gamepad1.x) {
                    robot.intakeRatio += 0.01;
                    sleep(50);
                } else if (gamepad1.dpad_down && gamepad1.x) {
                    robot.intakeRatio -= 0.01;
                    sleep(50);
                }
            }
            if (robot.use_dumper && !robot.isTesting) {
                if (gamepad2.right_bumper && gamepad2.left_bumper) {
                    lift_back_init(); // back to initial position for collecting glyph
                } else if (gamepad2.dpad_left) {
                    dumper_higher();
                } else if (gamepad2.dpad_right) {
                    dumper_lower();
                } else if ((gamepad1.dpad_down&&gamepad1.start) || (gamepad2.dpad_down&&gamepad2.start)) {
                    dumper_shake();
                }
                else if ((gamepad1.dpad_up&&!gamepad1.x) || gamepad2.dpad_up) {
                    dumper_up();
                    sleep(200);
                } else if ((gamepad1.dpad_down && !gamepad1.x) || gamepad2.dpad_down) {
                    dumper_down();
                }

                if ((gamepad2.right_bumper) && gamepad2.back) { // manual lift up
                    lift_up(true);
                } else if ((gamepad2.right_trigger>0.1) && gamepad2.back) { // force down
                    lift_down(true);
                } else if (gamepad2.right_bumper && !gamepad2.back) { // manual lift up
                    lift_up(false);
                } else if ((gamepad2.right_trigger > 0.1) && !gamepad2.back) { // manual down
                    lift_down(false);
                } else {
                    lift_stop();
                }
            }

            if (robot.use_newbot) {

                if(robot.isTesting){ //Allow to test individual movement

                    if (gamepad1.dpad_left) {
                        sleep(100);
                        TurnLeftD(0.5, 90.0);
                    }
                    if (gamepad1.dpad_right) {
                        sleep(100);
                        TurnRightD(0.5, 90.0);
                    }

                    if(gamepad1.left_trigger > 0.1){
                        robot.NB_LEFT_SV_DIFF -= 0.001;
                        sleep(100);
                    }

                    if(gamepad1.left_bumper){
                        robot.NB_LEFT_SV_DIFF += 0.001;
                        sleep(100);
                    }

                    if(gamepad1.right_trigger > 0.1){
                        robot.NB_RIGHT_SV_DIFF -= 0.001;
                        sleep(100);
                    }

                    if(gamepad1.right_bumper){
                        robot.NB_RIGHT_SV_DIFF += 0.001;
                        sleep(100);
                    }

                    if (gamepad1.b) {
                        StraightIn(0.5, 22);
                    }

                    if (gamepad1.a){
                        StraightIn(-0.5, 22);
                    }

                    if(gamepad1.x){
                        TurnLeftD(0.4, 90);
                    }
                    if(gamepad1.y){
                        TurnRightD(0.4, 90);
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
                    if (robot.use_newbot) {
                        robot.initialize_newbot();
                    }
                } // end isTesting
                else { //If not allowed to test servo positions, triggers do teleop spot turn
                    if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                        //if (Math.abs(gamepad1.right_stick_y) > 0.2 &&
                        //        Math.abs(gamepad1.right_stick_y) > 0.2 &&
                        //        Math.abs(gamepad1.left_stick_y - gamepad1.right_stick_y) > 0.4 ) {
                        // Swerve Turn
                        if (robot.cur_mode != SwerveDriveHardware.CarMode.TURN) {
                            change_swerve_pos(SwerveDriveHardware.CarMode.TURN);
                            // sleep(200);
                        }
                        // double pw_dir = (gamepad1.right_stick_y>0.1?-1.0:1.0);
                        double pw_dir = (gamepad1.right_stick_x > 0.1 ? 1.0 : -1.0);
                        // while (Math.abs(gamepad1.left_stick_y -gamepad1.right_stick_y) > 0.4) {
                        while (Math.abs(gamepad1.left_stick_x) > 0.1) {
                            robot.motorFrontLeft.setPower(robot.drivePowerRatio * pw_dir);
                            robot.motorFrontRight.setPower(-1 * robot.drivePowerRatio * pw_dir);
                            robot.motorBackLeft.setPower(robot.drivePowerRatio * pw_dir);
                            robot.motorBackRight.setPower(-1 * robot.drivePowerRatio * pw_dir);
                        }
                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    //} else if (Math.abs(gamepad1.left_stick_x)>0.2 &&
                    //        Math.abs(gamepad1.right_stick_x)>0.2 &&
                    //        Math.abs(gamepad1.left_stick_x+gamepad1.right_stick_x)>0.2) {
                        // crab move left / right
                        if (robot.cur_mode != SwerveDriveHardware.CarMode.CRAB) {
                            change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
                            sleep(100);
                        }
                        //double pw_dir = (gamepad1.left_stick_x>0.1?-1.0:1.0);
                        //while (Math.abs(gamepad1.left_stick_x+gamepad1.right_stick_x)>0.2) {
                        while (gamepad1.left_bumper || gamepad1.right_bumper) {
                            double pw_dir = (gamepad1.left_bumper ? 1.0:-1.0);
                            robot.motorFrontLeft.setPower(-robot.drivePowerRatio*pw_dir);
                            robot.motorFrontRight.setPower(robot.drivePowerRatio*pw_dir);
                            robot.motorBackLeft.setPower(-robot.drivePowerRatio*pw_dir);
                            robot.motorBackRight.setPower(robot.drivePowerRatio*pw_dir);
                        }
                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    } else if (Math.abs(gamepad1.left_stick_y)>0.1) {
                        if (robot.cur_mode != SwerveDriveHardware.CarMode.CAR) {
                            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
                            sleep(200);
                        }
                    }
                    else if (gamepad1.dpad_left){ // orbot left
                        if (robot.cur_mode != SwerveDriveHardware.CarMode.ORBIT) {
                            change_swerve_pos(SwerveDriveHardware.CarMode.ORBIT);
                            sleep(200);
                        }
                        while (gamepad1.dpad_left) {
                            robot.motorFrontLeft.setPower(-robot.drivePowerRatio);
                            robot.motorFrontRight.setPower(robot.drivePowerRatio);
                            robot.motorBackLeft.setPower(-robot.drivePowerRatio);
                            robot.motorBackRight.setPower(robot.drivePowerRatio);
                        }
                        // change_swerve_pos(robot.old_mode);
                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    } else if (gamepad1.dpad_right){ // orbit right
                        if (robot.cur_mode != SwerveDriveHardware.CarMode.ORBIT) {
                            change_swerve_pos(SwerveDriveHardware.CarMode.ORBIT);
                            sleep(200);
                        }
                        while (gamepad1.dpad_right) {
                            robot.motorFrontLeft.setPower(robot.drivePowerRatio);
                            robot.motorFrontRight.setPower(-robot.drivePowerRatio);
                            robot.motorBackLeft.setPower(robot.drivePowerRatio);
                            robot.motorBackRight.setPower(-robot.drivePowerRatio);
                        }
                        // change_swerve_pos(robot.old_mode);
                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }

                    if (gamepad1.a && gamepad1.y) {
                        robot.drivePowerRatio = 0.7;
                        sleep(20);

                    } else if (gamepad1.a && gamepad1.back){
                        robot.drivePowerRatio -=0.01;
                        if(robot.drivePowerRatio < 0.1){
                            robot.drivePowerRatio = 0.1;
                        }
                    }
                    else if (gamepad1.y && gamepad1.back){
                        robot.drivePowerRatio += 0.01;
                        if(robot.drivePowerRatio > 1.0){
                            robot.drivePowerRatio = 1.0;
                        }
                    } else if (gamepad1.y) { // slow mode
                        robot.drivePowerRatio = 0.5;
                    } else if (gamepad1.a) { // slow mode
                        robot.drivePowerRatio = 0.2;
                    }

                }

                if (robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                    if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                        calc_snake(gamepad1.left_trigger, gamepad1.right_trigger);
                    } else {
                        float left_x = 0, right_x = 0;
                        calc_snake(left_x, right_x);
                    }
                    snake_servo_adj();
                }

                correct_swerve_servos();

                //set_swerve_power(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);
                set_swerve_power(gamepad1.left_stick_y, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

                if (robot.cur_mode==SwerveDriveHardware.CarMode.CAR ||
                    robot.cur_mode==SwerveDriveHardware.CarMode.STRAIGHT) {
                    // use gamepad1.joy_stick_x to turn like Car
                    if ((adj_count++)%20==0)
                       car_servo_adj(gamepad1.left_stick_x);
                }
            } // end use_newbot

            if (robot.use_relic_slider) {
                // relic slider
                if (gamepad2.left_stick_y > 0.1) { // slide in
                    double pw = gamepad2.left_stick_y*gamepad2.left_stick_y;
                    if (!gamepad2.back)
                        pw *= 0.7;
                    relic_slider_in(pw, gamepad2.start); // push start to force slide in further
                } else if (gamepad2.left_stick_y < -0.1) { // slide out
                    double pw = gamepad2.left_stick_y*gamepad2.left_stick_y;
                    if (!gamepad2.back)
                        pw *= 0.85;
                    relic_slider_out(pw);
                } else {
                    relic_slider_stop();
                }
            }
            if (robot.use_relic_grabber) {
                // relic arm
                if (gamepad2.right_stick_y > 0.1) {
                    double cur_pos = robot.sv_relic_wrist.getPosition();
                    if (cur_pos < 0.99) {
                        robot.sv_relic_wrist.setPosition(cur_pos + 0.005);
                    }
                } else if (gamepad2.right_stick_y < -0.1) {
                    double cur_pos = robot.sv_relic_wrist.getPosition();
                    if (cur_pos > 0.01) {
                        robot.sv_relic_wrist.setPosition(cur_pos - 0.005);
                    }
                } else if (gamepad2.a && gamepad2.y) {
                    relic_arm_middle();
                } else if (gamepad2.y && !gamepad2.left_bumper) {
                    relic_arm_up();
                } else if (gamepad2.a && !gamepad2.left_bumper) {
                    relic_arm_down();
                }
                // relic grabber open/close
                if (gamepad2.b && !gamepad2.start) {
                    relic_grabber_close();
                } else if (gamepad2.x && gamepad2.back) {
                    // relic_grabber_release();
                    auto_relic_release();
                } else if (gamepad2.x && !gamepad2.dpad_right) {
                    relic_grabber_release();
                    // auto_relic_release();
                } else if (gamepad2.back && (gamepad2.right_stick_y < -0.1)) { // higher
                    relic_grabber_higher();
                } else if (gamepad2.back && (gamepad2.right_stick_y > 0.1)) { // lower
                    relic_grabber_lower();
                } else if (gamepad2.a && gamepad2.y) {
                    relic_arm_middle();
                } else if (gamepad2.y) {
                    relic_arm_up();
                } else if (gamepad2.a) {
                    relic_arm_down();
                }
            }
            show_telemetry();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            // robot.waitForTick(40);
        }
    }
}
