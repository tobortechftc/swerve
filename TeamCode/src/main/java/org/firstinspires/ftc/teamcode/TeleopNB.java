package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.SwerveSystem;

import static org.firstinspires.ftc.teamcode.hardware.SwerveSystem.CarMode.CAR;
import static org.firstinspires.ftc.teamcode.hardware.SwerveSystem.CarMode.CRAB;
import static org.firstinspires.ftc.teamcode.hardware.SwerveSystem.CarMode.ORBIT;
import static org.firstinspires.ftc.teamcode.hardware.SwerveSystem.CarMode.TURN;


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
        enable_hardware_for_teleop();
        // set_verbose();  // uncomment this line to debug

        init_and_test();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        start_init();
        robot.relicReachSystem.relic_arm_up(); // arm up to prevent intake gate collision

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // The left stick up/down moves the wheels forward and backwards, while the right stick left/right should rotate the wheels

            if(gamepad1.back && gamepad1.start){ // swap test/normalglyph_grabber_auto_open mode
                robot.isTesting = !robot.isTesting;
                sleep(100);
            }
            if (robot.intake.use_intake && !robot.isTesting) {
                if (gamepad1.x && gamepad1.dpad_right) {
                    robot.intake.intakeTurn(true);
                } else if (gamepad1.x && gamepad1.dpad_left) {
                    robot.intake.intakeTurn(false);
                } else if (gamepad1.x && gamepad1.a) {
                    robot.intake.correctGlyph(false);
                } else if (gamepad1.x || (gamepad2.left_bumper&&!gamepad2.right_bumper)) { // intake IN
                    robot.intake.intakeIn();
                } else if (gamepad1.b || (gamepad2.left_trigger > 0.1)) { // intake OUT
                    robot.intake.intakeOut();
                } else {
                    robot.intake.intakeStop();
                }
                if (gamepad1.dpad_up && gamepad1.x) {
                    robot.intake.intakeRatio += 0.01;
                    sleep(50);
                } else if (gamepad1.dpad_down && gamepad1.x) {
                    robot.intake.intakeRatio -= 0.01;
                    sleep(50);
                }
            }
//            if (robot.use_front_arm) {
//                if (gamepad2.back && gamepad2.dpad_down) {
//                    front_arm_sweep();
//                }
//            }
            if (robot.dumper.use_dumper && !robot.isTesting) {
                if (gamepad2.back && gamepad2.y) {
                    robot.intake.intakeBarWheelOut();
                } else if (gamepad2.back && gamepad2.a) {
                    robot.intake.intakeBarWheelIn();
                } else {
                    robot.intake.intakeBarWheelStop();
                }
                if (gamepad2.right_bumper && gamepad2.left_bumper) {
                    robot.dumper.lift_back_init(); // back to initial position for collecting glyph
                } else if (gamepad2.dpad_left && !gamepad2.y) {
                    // dumper_higher();
                    robot.intake.intakeGateUp();
                    sleep(50);
                } else if (gamepad2.dpad_right) {
                    // dumper_lower();
                    robot.intake.intakeGateDown();
                    sleep(50);
                } else if ((gamepad1.dpad_down&&gamepad1.start) || (gamepad2.dpad_down&&gamepad2.start)) {
                    robot.dumper.dumper_shake();
                }
                else if ((gamepad1.dpad_up&&!gamepad1.x) || gamepad2.dpad_up) {
                    robot.dumper.dumper_up();
                    boolean auto_up_down = false;
                    if (gamepad1.a) {
                        auto_up_down = true;
                    }
                    double pos = robot.dumper.sv_dumper.getPosition();
                    sleep(200);
                    if (auto_up_down && (Math.abs(pos-robot.dumper.SV_DUMPER_UP)<0.05)) {
                        robot.dumper.lift_up_and_down(true);
                    }
                } else if ((gamepad1.dpad_down && !gamepad1.x) || gamepad2.dpad_down) {
                    robot.dumper.dumper_down(true);
                }

                if ((gamepad2.right_bumper) && gamepad2.back) { // manual lift up
                    robot.dumper.lift_up(true);
                } else if ((gamepad2.right_trigger>0.1) && gamepad2.back) { // force down
                    robot.dumper.lift_down(true);
                } else if (gamepad2.right_bumper && !gamepad2.back) { // manual lift up
                    robot.dumper.lift_up(false);
                } else if ((gamepad2.right_trigger > 0.1) && !gamepad2.back) { // manual down
                    robot.dumper.lift_down(false);
                } else {
                    robot.dumper.lift_stop();
                }
            }

            if (robot.swerve.use_newbot) {

                if(robot.isTesting){ //Allow to test individual movement

                    if (gamepad1.dpad_left) {
                        sleep(100);
                        robot.swerve.TurnLeftD(0.5, 90.0);
                    }
                    if (gamepad1.dpad_right) {
                        sleep(100);
                        robot.swerve.TurnRightD(0.5, 90.0);
                    }

                    if(gamepad1.left_trigger > 0.1){
                        robot.swerve.NB_LEFT_SV_DIFF -= 0.001;
                        sleep(100);
                    }

                    if(gamepad1.left_bumper){
                        robot.swerve.NB_LEFT_SV_DIFF += 0.001;
                        sleep(100);
                    }

                    if(gamepad1.right_trigger > 0.1){
                        robot.swerve.NB_RIGHT_SV_DIFF -= 0.001;
                        sleep(100);
                    }

                    if(gamepad1.right_bumper){
                        robot.swerve.NB_RIGHT_SV_DIFF += 0.001;
                        sleep(100);
                    }

                    if (gamepad1.b) {
                        robot.swerve.StraightIn(0.5, 22);
                    }

                    if (gamepad1.a){
                        robot.swerve.StraightIn(-0.5, 22);
                    }

                    if(gamepad1.x){
                        robot.swerve.TurnLeftD(0.4, 90);
                    }
                    if(gamepad1.y){
                        robot.swerve.TurnRightD(0.4, 90);
                    }

                    if(gamepad1.start){
                        if(!(robot.swerve.cur_mode == SwerveSystem.CarMode.CRAB)){// If in any other mode, switch to crab
                            robot.swerve.change_swerve_pos(CRAB);
                        }
                        else{ //Return from snake to previous drive mode
                            robot.swerve.change_swerve_pos(robot.swerve.old_mode);
                        }
                        sleep(400);
                    }
                    if (robot.swerve.use_newbot) {
                        robot.swerve.initialize_newbot();
                    }
                } // end isTesting
                else { //If not allowed to test servo positions, triggers do teleop spot turn
                    if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                        //if (Math.abs(gamepad1.right_stick_y) > 0.2 &&
                        //        Math.abs(gamepad1.right_stick_y) > 0.2 &&
                        //        Math.abs(gamepad1.left_stick_y - gamepad1.right_stick_y) > 0.4 ) {
                        // Swerve Turn
                        if (robot.swerve.cur_mode != SwerveSystem.CarMode.TURN) {
                            robot.swerve.change_swerve_pos(TURN);
                            // sleep(200);
                        }
                        // double pw_dir = (gamepad1.right_stick_y>0.1?-1.0:1.0);
                        double pw_dir = (gamepad1.right_stick_x > 0.1 ? -1.0 : 1.0);
                        // while (Math.abs(gamepad1.left_stick_y -gamepad1.right_stick_y) > 0.4) {
                        while (Math.abs(gamepad1.right_stick_x) > 0.1) {
                            robot.swerve.motorFrontLeft.setPower(robot.swerve.drivePowerRatio * pw_dir);
                            robot.swerve.motorFrontRight.setPower(-1 * robot.swerve.drivePowerRatio * pw_dir);
                            robot.swerve.motorBackLeft.setPower(robot.swerve.drivePowerRatio * pw_dir);
                            robot.swerve.motorBackRight.setPower(-1 * robot.swerve.drivePowerRatio * pw_dir);
                        }
                        robot.swerve.driveTT(0,0);
                    } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    //} else if (Math.abs(gamepad1.left_stick_x)>0.2 &&
                    //        Math.abs(gamepad1.right_stick_x)>0.2 &&
                    //        Math.abs(gamepad1.left_stick_x+gamepad1.right_stick_x)>0.2) {
                        // crab move left / right
                        if (robot.swerve.cur_mode != SwerveSystem.CarMode.CRAB) {
                            robot.swerve.change_swerve_pos(CRAB);
                            sleep(100);
                        }
                        //double pw_dir = (gamepad1.left_stick_x>0.1?-1.0:1.0);
                        //while (Math.abs(gamepad1.left_stick_x+gamepad1.right_stick_x)>0.2) {
                        while (gamepad1.left_bumper || gamepad1.right_bumper) {
                            double pw_dir = (gamepad1.left_bumper ? 1.0:-1.0);
                            robot.swerve.motorFrontLeft.setPower(-robot.swerve.drivePowerRatio*pw_dir);
                            robot.swerve.motorFrontRight.setPower(robot.swerve.drivePowerRatio*pw_dir);
                            robot.swerve.motorBackLeft.setPower(-robot.swerve.drivePowerRatio*pw_dir);
                            robot.swerve.motorBackRight.setPower(robot.swerve.drivePowerRatio*pw_dir);
                        }
                        robot.swerve.driveTT(0,0);
                    } else if (Math.abs(gamepad1.left_stick_y)>0.1) {
                        if (robot.swerve.cur_mode != SwerveSystem.CarMode.CAR) {
                            robot.swerve.change_swerve_pos(CAR);
                            sleep(200);
                        }
                    }
                    else if (gamepad1.dpad_left && !gamepad1.x){ // orbot left
                        if (robot.swerve.cur_mode != SwerveSystem.CarMode.ORBIT) {
                            robot.swerve.change_swerve_pos(ORBIT);
                            sleep(200);
                        }
                        while (gamepad1.dpad_left && !gamepad1.x) {
                            robot.swerve.motorFrontLeft.setPower(-robot.swerve.drivePowerRatio);
                            robot.swerve.motorFrontRight.setPower(robot.swerve.drivePowerRatio);
                            robot.swerve.motorBackLeft.setPower(-robot.swerve.drivePowerRatio);
                            robot.swerve.motorBackRight.setPower(robot.swerve.drivePowerRatio);
                        }
                        // change_swerve_pos(robot.old_mode);
                        robot.swerve.driveTT(0,0);
                    } else if (gamepad1.dpad_right && !gamepad1.x){ // orbit right
                        if (robot.swerve.cur_mode != SwerveSystem.CarMode.ORBIT) {
                            robot.swerve.change_swerve_pos(ORBIT);
                            sleep(200);
                        }
                        while (gamepad1.dpad_right && !gamepad1.x) {
                            robot.swerve.motorFrontLeft.setPower(robot.swerve.drivePowerRatio);
                            robot.swerve.motorFrontRight.setPower(-robot.swerve.drivePowerRatio);
                            robot.swerve.motorBackLeft.setPower(robot.swerve.drivePowerRatio);
                            robot.swerve.motorBackRight.setPower(-robot.swerve.drivePowerRatio);
                        }
                        // change_swerve_pos(robot.old_mode);
                        robot.swerve.driveTT(0,0);
                    }

                    if (gamepad1.a && gamepad1.y) {
                        robot.swerve.drivePowerRatio = 0.9;
                        sleep(20);

                    } else if (gamepad1.a && gamepad1.back){
                        robot.swerve.drivePowerRatio -=0.01;
                        if(robot.swerve.drivePowerRatio < 0.1){
                            robot.swerve.drivePowerRatio = 0.1;
                        }
                    }
                    else if (gamepad1.y && gamepad1.back){
                        robot.swerve.drivePowerRatio += 0.01;
                        if(robot.swerve.drivePowerRatio > 1.0){
                            robot.swerve.drivePowerRatio = 1.0;
                        }
                    } else if (gamepad1.y) { // slow mode
                        robot.swerve.drivePowerRatio = 0.7;
                    } else if (gamepad1.a && !gamepad1.x && !gamepad1.dpad_up) { // slow mode
                        robot.swerve.drivePowerRatio = 0.2;
                    }

                }

                if (robot.swerve.cur_mode == SwerveSystem.CarMode.CAR) {
                    if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
                        robot.swerve.calc_snake(gamepad1.left_trigger, gamepad1.right_trigger);
                    } else {
                        float left_x = 0, right_x = 0;
                        robot.swerve.calc_snake(left_x, right_x);
                    }
                    robot.swerve.snake_servo_adj();
                }

                robot.swerve.correct_swerve_servos();

                //set_swerve_power(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);
                robot.swerve.set_swerve_power(gamepad1.left_stick_y, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

                if (robot.swerve.cur_mode== SwerveSystem.CarMode.CAR ||
                    robot.swerve.cur_mode== SwerveSystem.CarMode.STRAIGHT) {
                    // use gamepad1.joy_stick_x to turn like Car
                    if ((adj_count++)%20==0)
                        robot.swerve.car_servo_adj(gamepad1.left_stick_x);
                }
            } // end use_newbot

            if (robot.relicReachSystem.use_relic_slider) {
                // relic slider
                if (gamepad2.left_stick_y > 0.2) { // slide in
                    double pw = gamepad2.left_stick_y*gamepad2.left_stick_y;
                    if (!gamepad2.back)
                        pw *= 0.9;
                    robot.relicReachSystem.relic_slider_in(pw, gamepad2.start); // push start to force slide in further
                } else if (gamepad2.left_stick_y < -0.2) { // slide out
                    double pw = gamepad2.left_stick_y*gamepad2.left_stick_y;
                    if (!gamepad2.back)
                        pw *= 0.99;
                    robot.relicReachSystem.relic_slider_out(pw);
                } else if (gamepad2.left_stick_x < -0.2) { // slide out slowly
                    robot.relicReachSystem.relic_slider_out(0.3);
                } else if (gamepad2.left_stick_x > 0.2) { // slide in slowly
                    robot.relicReachSystem.relic_slider_in(0.2, false);
                } else {
                    robot.relicReachSystem.relic_slider_stop();
                }
            }
            if (robot.relicReachSystem.use_relic_grabber) {
                // relic arm
                if (gamepad2.right_stick_y < -0.1) {
                    double cur_pos = robot.relicReachSystem.sv_relic_wrist.getPosition();
                    if (cur_pos < 0.99) {
                        robot.relicReachSystem.sv_relic_wrist.setPosition(cur_pos + 0.005);
                    }
                } else if (gamepad2.right_stick_y > 0.1) {
                    double cur_pos = robot.relicReachSystem.sv_relic_wrist.getPosition();
                    if (cur_pos > 0.01) {
                        robot.relicReachSystem.sv_relic_wrist.setPosition(cur_pos - 0.005);
                    }
                } else if (gamepad2.a && gamepad2.y) {
                    robot.relicReachSystem.relic_arm_middle();
                } else if (gamepad2.y && !gamepad2.left_bumper && !gamepad2.back) {
                    if (robot.relicReachSystem.use_relic_elbow) {
                        robot.relicReachSystem.relic_elbow_up_auto();
                    } else {
                        robot.relicReachSystem.relic_arm_up();
                    }
                } else if (gamepad2.a && !gamepad2.left_bumper && !gamepad2.back) {
                    if (robot.relicReachSystem.use_relic_elbow) {
                        robot.relicReachSystem.relic_elbow_down_auto();
                    } else {
                        robot.relicReachSystem.relic_arm_down();
                    }
                }
                // relic grabber open/close
                if (gamepad2.b && !gamepad2.start) {
                    robot.relicReachSystem.relic_grabber_close();
                } else if (gamepad2.x && gamepad2.back) {
                    // rrxx__relic_grabber_release();
                    auto_relic_release();
                } else if (gamepad2.x && !gamepad2.dpad_right) {
                    robot.relicReachSystem.relic_grabber_release();
                    // auto_relic_release();
                } else if (gamepad2.back && (gamepad2.right_stick_y < -0.1)) { // higher
                    if (robot.relicReachSystem.use_relic_elbow)
                        robot.relicReachSystem.relic_elbow_higher();
                    else
                        robot.relicReachSystem.relic_grabber_higher();
                } else if (gamepad2.back && (gamepad2.right_stick_y > 0.1)) { // lower
                    if (robot.relicReachSystem.use_relic_elbow)
                        robot.relicReachSystem.relic_elbow_lower();
                    else
                        robot.relicReachSystem.relic_grabber_lower();
                }
            }
            show_telemetry();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            // robot.waitForTick(40);
        }
        stop_tobot();
    }
}
