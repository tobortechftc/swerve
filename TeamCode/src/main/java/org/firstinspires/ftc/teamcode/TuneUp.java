package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TuneUp", group="Test")
public class TuneUp extends SwerveUtilLOP {

    /* Declare OpMode members. */
    static double INCREMENT = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
//        /* Initialize the hardware variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.servo_tune_up = true; // enable servo tune up
//        robot.use_swerve = false;
//        robot.use_newbot = true;
//        robot.use_newbot_v2= true;
//        robot.use_dumper = true;
//        robot.use_dumper_gate = true;
//        robot.use_intake = true;
//        robot.use_imu = true;
//        robot.use_Vuforia = false;
//        robot.use_camera = false;
//        // robot.use_color_sensor = true;
//        robot.use_arm = true;
//        robot.use_glyph_grabber = false;
//        robot.relicReachSystem.use_relic_grabber = true;
//        robot.relicReachSystem.use_relic_elbow = false;
//        robot.relicReachSystem.use_relic_slider = true;
//        robot.use_test_servo = false;
//        robot.use_test_motor = false;
//        robot.use_range_sensor = true;
//        robot.use_proximity_sensor = true;
//        robot.use_front_arm = false;
//        robot.shxx.use_verbose = true;
//
//        init_and_test();
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("My name is Tobot.", "Need tune-up?");
//        telemetry.update();
//
//        int num_servos = 17;
//        int cur_sv_ix = 0;
//        boolean show_all = true;
//        double cs_val = 0;
//        Servo[] sv_list = {
//                robot.servoFrontLeft,
//                robot.servoFrontRight,
//                robot.servoBackLeft,
//                robot.servoBackRight,
//                robot.sv_shoulder,
//                robot.sv_elbow,
//                robot.sv_left_arm,
//                robot.sv_right_arm,
//                robot.sv_glyph_grabber_bottom,
//                robot.sv_glyph_grabber_top,
//                robot.relicReachSystem.sv_relic_grabber,
//                robot.relicReachSystem.sv_relic_wrist,
//                robot.relicReachSystem.sv_relic_elbow,
//                robot.sv_dumper,
//                robot.sv_front_arm,
//                robot.sv_intake_gate,
//                robot.sv_dumper_gate,
//                robot.sv_jkicker
//        };
//        String [] sv_names = {
//                "FrontLeft",
//                "FrontRight",
//                "BackLeft",
//                "BackRight",
//                "sv_shoulder",
//                "sv_elbow",
//                "sv_left_arm",
//                "sv_right_arm",
//                "sv_gg_bottom",
//                "sv_gg_top",
//                "rrxx__sv_relic_grabber",
//                "rrxx__sv_relic_wrist",
//                "rrxx__sv_relic_elbow",
//                "sv_dumper",
//                "sv_front_arm",
//                "sv_intake_gate",
//                "sv_dumper_gate",
//                "sv_jkicker"
//        };
//
//        num_servos = sv_list.length;
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        start_init();
//        while (cur_sv_ix < num_servos && sv_list[cur_sv_ix] == null) {
//            cur_sv_ix++;
//        }
//        if (cur_sv_ix==num_servos) cur_sv_ix = 0; // no servo available
//
//        if (robot.use_glyph_grabber) {
//            glyph_grabber_auto_open();
//        }
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//            if (gamepad1.left_stick_y<-0.1) {
//                INCREMENT += 0.001;
//                if (INCREMENT>0.5) INCREMENT = 0.5;
//            } else if (gamepad1.left_stick_y>0.1) {
//                INCREMENT -= 0.001;
//                if (INCREMENT<0.001) INCREMENT = 0.001;
//            }
//
//            if (robot.use_arm && !robot.use_newbot) {
//                if (gamepad2.dpad_down) {
//                    double pos = robot.sv_shoulder.getPosition();
//                    if (pos >= INCREMENT) {
//                        robot.sv_shoulder.setPosition(pos - INCREMENT);
//                    }
//                }
//                if (gamepad2.dpad_up) {
//                    double pos = robot.sv_shoulder.getPosition();
//                    if (pos <= (1 - INCREMENT)) {
//                        robot.sv_shoulder.setPosition(pos + INCREMENT);
//                    }
//                }
//                if (gamepad2.dpad_right) {
//                    double pos = robot.sv_elbow.getPosition();
//                    if (pos <= (1 - INCREMENT)) {
//                        robot.sv_elbow.setPosition(pos + INCREMENT);
//                    }
//                }
//                if (gamepad2.dpad_left) {
//                    double pos = robot.sv_elbow.getPosition();
//                    if (pos >= INCREMENT) {
//                        robot.sv_elbow.setPosition(pos - INCREMENT);
//                    }
//                }
//                if (gamepad2.a) {
//                    robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
//                }
//                if (gamepad2.y) {
//                    robot.sv_elbow.setPosition(robot.SV_ELBOW_UP);
//                }
//                if (gamepad2.x) {
//                    robot.sv_shoulder.setPosition(robot.SV_SHOULDER_LEFT_3);
//                }
//                if (gamepad2.b) {
//                    robot.sv_shoulder.setPosition(robot.SV_SHOULDER_RIGHT_3);
//                }
//                if (gamepad2.left_bumper) {
//                    robot.sv_shoulder.setPosition(robot.SV_SHOULDER_DOWN);
//                }
//            }
//            if (robot.use_newbot_v2) {
//                if (gamepad2.dpad_left && gamepad2.y) {
//                    intakeBarWheelOut();
//                } else if (gamepad2.dpad_left && gamepad2.a) {
//                    intakeBarWheelIn();
//                } else if (gamepad2.dpad_right && gamepad2.y) {
//                    intakeBarWheelOutP(0.5);
//                } else if (gamepad2.dpad_right && gamepad2.a) {
//                    intakeBarWheelInP(0.5);
//                } else {
//                    intakeBarWheelStop();
//                }
//            }
//            if (robot.use_glyph_grabber) {
//                if (gamepad1.dpad_down) {
//                    double pos = robot.sv_glyph_grabber_bottom.getPosition();
//                    if (pos >= INCREMENT) {
//                        robot.sv_glyph_grabber_bottom.setPosition(pos - INCREMENT);
//                    }
//                }
//                if (gamepad1.dpad_up) {
//                    double pos = robot.sv_glyph_grabber_bottom.getPosition();
//                    if (pos <= (1 - INCREMENT)) {
//                        robot.sv_glyph_grabber_bottom.setPosition(pos + INCREMENT);
//                    }
//                }
//            }
//
//            if(gamepad1.back && gamepad1.start){ // swap test/normalglyph_grabber_auto_open mode
//                robot.isTesting = !robot.isTesting;
//                sleep(100);
//            }
//
//            if (robot.use_newbot) { // newbot related control
//                if (robot.use_dumper && !robot.isTesting) {
//                    if (gamepad1.dpad_up && gamepad1.a) {
//                        driveTTSnake(-0.3, (float) 1.0, true); sleep(700); stop_chassis();
//                    } else if (gamepad1.dpad_up && gamepad1.b) {
//                        alignBoxEdge();
//                    } else if (gamepad1.dpad_up && gamepad1.y) {
//                        deliverGlyph();
//                    } else if (gamepad1.dpad_up && gamepad1.x) {
//                        grabAndDump(true, true);
//                    }
//                }
//              /*  if (robot.use_newbot_v2) {
//                    if (gamepad2.bumperleft) {
//                        robot.sv_intake_gate.setPosition(robot.sv_intake_gate.getPosition() + 0.01);
//                    }
//                } */
//                if(robot.isTesting) { //Allow to test individual movement
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
//                    //Start of Crab adjust code
//                    if(gamepad1.y) { // FL
//                        if (gamepad1.dpad_up){
//                            robot.NB_CRAB_DIFF_INC_FL += INCREMENT;
//                            robot.needsUpdate = true;
//                        }
//
//                        else if (gamepad1.dpad_down){
//                            robot.NB_CRAB_DIFF_INC_FL -= INCREMENT;
//                            robot.needsUpdate = true;
//                        }
//                    }
//                    else if (gamepad1.x){ // BL
//                        if (gamepad1.dpad_up){
//                            robot.NB_CRAB_DIFF_INC_BL += INCREMENT;
//                            robot.needsUpdate = true;
//                        }
//
//                        else if (gamepad1.dpad_down){
//                            robot.NB_CRAB_DIFF_INC_BL -= INCREMENT;
//                            robot.needsUpdate = true;
//                        }
//                    }
//                    else if (gamepad1.b){ // FR
//                        if (gamepad1.dpad_up){
//                            robot.NB_CRAB_DIFF_DEC_FR += INCREMENT;
//                            robot.needsUpdate = true;
//                        }
//
//                        else if (gamepad1.dpad_down){
//                            robot.NB_CRAB_DIFF_DEC_FR -= INCREMENT;
//                            robot.needsUpdate = true;
//                        }
//                    }
//                    else if (gamepad1.a){
//                        if (gamepad1.dpad_up){
//                            robot.NB_CRAB_DIFF_DEC_BR += INCREMENT;
//                            robot.needsUpdate = true;
//                        }
//
//                        else if (gamepad1.dpad_down){
//                            robot.NB_CRAB_DIFF_DEC_BR -= INCREMENT;
//                            robot.needsUpdate = true;
//                        }
//                    }
//                    //End of Crab adjust code
//
//                    if (gamepad1.back && gamepad1.dpad_left){
//                        TurnLeftD(0.4, 180);
//                    } else if(gamepad1.start && gamepad1.dpad_left){
//                        double tar_heading = imu_heading()+180;
//                        TurnLeftD(0.4, 180);
//                        if (tar_heading>=180) {
//                            tar_heading -= 360;
//                        }
//                        alignUsingIMU(0.2, tar_heading);
//                    }
//                    else if(gamepad1.back && gamepad1.dpad_right){
//                        TurnRightD(0.4, 180);
//                    }
//                    else if(gamepad1.start && gamepad1.dpad_right){
//                        double tar_heading = imu_heading()-180;
//                        TurnRightD(0.4, 180);
//                        if (tar_heading<=-180) {
//                            tar_heading += 360;
//                        }
//                        alignUsingIMU(0.2, tar_heading);
//                    }
//                    else if(gamepad1.dpad_left){
//                        TurnLeftD(0.4, 90);
//                    }
//                    else if(gamepad1.dpad_right){
//                        TurnRightD(0.4, 90);
//                    } else if (gamepad1.back && gamepad1.b) {
//                        StraightIn(0.5, 22);
//                    } else if (gamepad1.back && gamepad1.a){
//                        StraightIn(-0.5, 22);
//                    }
//                    else if (gamepad1.start){
//                        if(!(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB)){// If in any other mode, switch to crab
//                            change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
//                        }
//                        else{ //Return from snake to previous drive mode
//                            change_swerve_pos(robot.old_mode);
//                        }
//                        sleep(400);
//                    }
//                    if (robot.use_newbot) {
//                        robot.initialize_newbot();
//                        if(robot.needsUpdate){
//                            change_swerve_pos(robot.cur_mode);
//                            robot.needsUpdate = false;
//                        }
//                    }
//                } // end isTesting
//            } // end use_swerve
//            if (robot.sv_bar_wheel!=null) {
//                if (gamepad2.y) {
//                    cs_val += 0.1;
//                    if (cs_val>1) cs_val=1;
//                } else if (gamepad2.a) {
//                    cs_val -= 0.1;
//                    if (cs_val<-1) cs_val=-1;
//                }
//                robot.sv_bar_wheel.setPower(cs_val);
//            }
//            if (robot.use_test_motor) {
//                if (gamepad1.x) {
//                   if (robot.is_gg_upside_down) {
//                       test_rotate(-0.4);
//                   } else {
//                       test_rotate(0.4);
//                   }
//                }
//                if (gamepad1.b) {
//                    rotate_refine();
//                }
//            }
//            if (robot.use_glyph_grabber) {
//                if (gamepad1.left_bumper) {
//                    glyph_grabber_auto_open();
//                } else if (gamepad1.left_trigger > 0.1) {
//                    // 1. glyph grabber auto close down grabber
//                    // 2. if not upside down yet, glyph grabber auto rotates 180 degrees
//                    glyph_grabber_auto_close(false,false);
//                    if (!robot.is_gg_upside_down) {
//                        sleep(1000);
//                        glyph_grabber_auto_rotate(0.4);
//                    }
//                }
//                if (gamepad1.right_bumper) {
//                    glyph_slider_up();
//                } else if (gamepad1.right_trigger > 0.1) {
//                    glyph_slider_down();
//                } else {
//                    glyph_slider_stop();
//                }
//                if (gamepad1.dpad_down) {
//                    rotate_refine();
//                } else if (gamepad1.dpad_left) {
//                    glyph_grabber_all_close();
//                    glyph_grabber_auto_rotate(0.4);
//                }
//            }
//            if (robot.servo_tune_up && !robot.isTesting) {
//                if (gamepad1.back && gamepad1.a) {
//                    show_all = !show_all;
//                    sleep(50);
//                } else if (gamepad1.a && (sv_list[cur_sv_ix] != null)) {
//                    double pos = sv_list[cur_sv_ix].getPosition();
//                    if (pos <= (1 - INCREMENT)) {
//                        sv_list[cur_sv_ix].setPosition(pos + INCREMENT);
//                    }
//                    sleep(50);
//                } else if (gamepad1.y && (sv_list[cur_sv_ix] != null)) {
//                    double pos = sv_list[cur_sv_ix].getPosition();
//                    if (pos >= INCREMENT) {
//                        sv_list[cur_sv_ix].setPosition(pos - INCREMENT);
//                    }
//                    sleep(50);
//                }
//
//                if (gamepad1.x) {
//                    cur_sv_ix--;
//                    if (cur_sv_ix < 0) cur_sv_ix = num_servos - 1;
//                    int count = 0;
//                    while (sv_list[cur_sv_ix] == null && cur_sv_ix >= 0 && count < 20) {
//                        cur_sv_ix--;
//                        if (cur_sv_ix < 0) cur_sv_ix = num_servos - 1;
//                        count++;
//                    }
//                    gamepad1.reset();
//                    sleep(400);
//                } else if (gamepad1.b) {
//                    cur_sv_ix++;
//                    if (cur_sv_ix >= num_servos) cur_sv_ix = 0;
//                    int count = 0;
//                    while (cur_sv_ix < num_servos && sv_list[cur_sv_ix] == null && count < 20) {
//                        cur_sv_ix++;
//                        if (cur_sv_ix >= num_servos) cur_sv_ix = 0;
//                        count++;
//                    }
//                    gamepad1.reset();
//                    sleep(400);
//                }
//            }
//            telemetry.addData("0. GP1:", "x/b:sv sel, y/a:+/-%4.3f(ix=%d)", INCREMENT, cur_sv_ix);
//            telemetry.addData("0.1 cvservo =", "%2.1f", cs_val);
//            if (show_all) {
//                for (int i = 0; i < num_servos; i++) {
//                    if (sv_list[i] != null) {
//                        telemetry.addData("6.", "%d: %s sv-port %d = %5.4f",
//                                i, sv_names[i], sv_list[i].getPortNumber(), sv_list[i].getPosition());
//                    }
//                }
//            } else if (sv_list[cur_sv_ix] != null) {
//                telemetry.addData("7. Tune-up servo", " %s (ix=%d) = %5.4f",
//                        sv_names[cur_sv_ix], cur_sv_ix, sv_list[cur_sv_ix].getPosition());
//            } else {
//                telemetry.addLine("7. No active servo to tune-up.");
//            }
//            if (robot.use_color_sensor) {
//                double l_d = (robot.use_newbot_v2?0:calcDelta(true));
//                double r_d = calcDelta(false);
//                telemetry.addData("5.1 Color delta", "l_d=%1.0f, r_d=%1.0f", l_d,r_d);
//            }
//            show_telemetry();
//            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//            // robot.waitForTick(40);
//        }
    }

}


