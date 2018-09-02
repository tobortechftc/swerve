package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.SwerveSystem;


@TeleOp(name="TuneUp", group="Test")
public class TuneUp extends SwerveUtilLOP {

    /* Declare OpMode members. */
    static double INCREMENT = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        enable_hardware_for_teleop();
        robot.servo_tune_up = true; // enable servo tune up
        robot.swerve.use_imu = true;
        robot.swerve.use_range_sensor = true;
        robot.swerve.use_proximity_sensor = true;
        // set_verbose();  // uncomment this line to debug

        init_and_test();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("My name is Tobot.", "Need tune-up?");
        telemetry.update();

        int cur_sv_ix = 0;
        boolean show_all = true;
        double cs_val = 0;
        Servo[] sv_list = {
                robot.swerve.servoFrontLeft,
                robot.swerve.servoFrontRight,
                robot.swerve.servoBackLeft,
                robot.swerve.servoBackRight,
                robot.jewel.sv_shoulder,
                robot.jewel.sv_elbow,
                robot.jewel.sv_left_arm,
                robot.jewel.sv_right_arm,
                robot.relicReachSystem.sv_relic_grabber,
                robot.relicReachSystem.sv_relic_wrist,
                robot.relicReachSystem.sv_relic_elbow,
                robot.dumper.sv_dumper,
                robot.intake.sv_intake_gate,
                robot.dumper.sv_dumper_gate,
                robot.jewel.sv_jkicker
        };
        String [] sv_names = {
                "FrontLeft",
                "FrontRight",
                "BackLeft",
                "BackRight",
                "sv_shoulder",
                "sv_elbow",
                "sv_left_arm",
                "sv_right_arm",
                "sv_gg_bottom",
                "sv_gg_top",
                "rrxx__sv_relic_grabber",
                "rrxx__sv_relic_wrist",
                "rrxx__sv_relic_elbow",
                "sv_dumper",
                "sv_front_arm",
                "sv_intake_gate",
                "sv_dumper_gate",
                "sv_jkicker"
        };

        int num_servos = sv_list.length;
        boolean needsUpdate = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        start_init();
        while (cur_sv_ix < num_servos && sv_list[cur_sv_ix] == null) {
            cur_sv_ix++;
        }
        if (cur_sv_ix==num_servos) cur_sv_ix = 0; // no servo available

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_stick_y<-0.1) {
                INCREMENT += 0.001;
                if (INCREMENT>0.5) INCREMENT = 0.5;
            } else if (gamepad1.left_stick_y>0.1) {
                INCREMENT -= 0.001;
                if (INCREMENT<0.001) INCREMENT = 0.001;
            }

            {
                if (gamepad2.dpad_left && gamepad2.y) {
                    robot.intake.intakeBarWheelOut();
                } else if (gamepad2.dpad_left && gamepad2.a) {
                    robot.intake.intakeBarWheelIn();
                } else {
                    robot.intake.intakeBarWheelStop();
                }
            }

            if(gamepad1.back && gamepad1.start){ // swap test/normalglyph_grabber_auto_open mode
                robot.isTesting = !robot.isTesting;
                sleep(100);
            }

            if (robot.swerve.use_swerve) { // newbot related control
              /*  if (robot.use_newbot_v2) {
                    if (gamepad2.bumperleft) {
                        robot.sv_intake_gate.setPosition(robot.sv_intake_gate.getPosition() + 0.01);
                    }
                } */
                if(robot.isTesting) { //Allow to test individual movement

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
                    //Start of Crab adjust code
                    if(gamepad1.y) { // FL
                        if (gamepad1.dpad_up){
                            robot.swerve.NB_CRAB_DIFF_INC_FL += INCREMENT;
                            needsUpdate = true;
                        }

                        else if (gamepad1.dpad_down){
                            robot.swerve.NB_CRAB_DIFF_INC_FL -= INCREMENT;
                            needsUpdate = true;
                        }
                    }
                    else if (gamepad1.x){ // BL
                        if (gamepad1.dpad_up){
                            robot.swerve.NB_CRAB_DIFF_INC_BL += INCREMENT;
                            needsUpdate = true;
                        }

                        else if (gamepad1.dpad_down){
                            robot.swerve.NB_CRAB_DIFF_INC_BL -= INCREMENT;
                            needsUpdate = true;
                        }
                    }
                    else if (gamepad1.b){ // FR
                        if (gamepad1.dpad_up){
                            robot.swerve.NB_CRAB_DIFF_DEC_FR += INCREMENT;
                            needsUpdate = true;
                        }

                        else if (gamepad1.dpad_down){
                            robot.swerve.NB_CRAB_DIFF_DEC_FR -= INCREMENT;
                            needsUpdate = true;
                        }
                    }
                    else if (gamepad1.a){
                        if (gamepad1.dpad_up){
                            robot.swerve.NB_CRAB_DIFF_DEC_BR += INCREMENT;
                            needsUpdate = true;
                        }

                        else if (gamepad1.dpad_down){
                            robot.swerve.NB_CRAB_DIFF_DEC_BR -= INCREMENT;
                            needsUpdate = true;
                        }
                    }
                    //End of Crab adjust code

                    if (gamepad1.back && gamepad1.dpad_left){
                        robot.swerve.TurnLeftD(0.4, 180);
                    } else if(gamepad1.start && gamepad1.dpad_left){
                        double tar_heading = robot.swerve.imu_heading()+180;
                        robot.swerve.TurnLeftD(0.4, 180);
                        if (tar_heading>=180) {
                            tar_heading -= 360;
                        }
                        robot.swerve.alignUsingIMU(0.2, tar_heading);
                    }
                    else if(gamepad1.back && gamepad1.dpad_right){
                        robot.swerve.TurnRightD(0.4, 180);
                    }
                    else if(gamepad1.start && gamepad1.dpad_right){
                        double tar_heading = robot.swerve.imu_heading()-180;
                        robot.swerve.TurnRightD(0.4, 180);
                        if (tar_heading<=-180) {
                            tar_heading += 360;
                        }
                        robot.swerve.alignUsingIMU(0.2, tar_heading);
                    }
                    else if(gamepad1.dpad_left){
                        robot.swerve.TurnLeftD(0.4, 90);
                    }
                    else if(gamepad1.dpad_right){
                        robot.swerve.TurnRightD(0.4, 90);
                    } else if (gamepad1.back && gamepad1.b) {
                        robot.swerve.StraightIn(0.5, 22);
                    } else if (gamepad1.back && gamepad1.a){
                        robot.swerve.StraightIn(-0.5, 22);
                    }
                    else if (gamepad1.start){
                        if(!(robot.swerve.cur_mode == SwerveSystem.CarMode.CRAB)){// If in any other mode, switch to crab
                            robot.swerve.change_swerve_pos(SwerveSystem.CarMode.CRAB);
                        }
                        else{ //Return from snake to previous drive mode
                            robot.swerve.change_swerve_pos(robot.swerve.old_mode);
                        }
                        sleep(400);
                    }
                    {
                        robot.swerve.initialize_newbot();
                        if(needsUpdate){
                            robot.swerve.change_swerve_pos(robot.swerve.cur_mode);
                            needsUpdate = false;
                        }
                    }
                } // end isTesting
            } // end use_swerve
            if (robot.intake.sv_bar_wheel!=null) {
                if (gamepad2.y) {
                    cs_val += 0.1;
                    if (cs_val>1) cs_val=1;
                } else if (gamepad2.a) {
                    cs_val -= 0.1;
                    if (cs_val<-1) cs_val=-1;
                }
                robot.intake.sv_bar_wheel.setPower(cs_val);
            }

            if (robot.servo_tune_up && !robot.isTesting) {
                if (gamepad1.back && gamepad1.a) {
                    show_all = !show_all;
                    sleep(50);
                } else if (gamepad1.a && (sv_list[cur_sv_ix] != null)) {
                    double pos = sv_list[cur_sv_ix].getPosition();
                    if (pos <= (1 - INCREMENT)) {
                        sv_list[cur_sv_ix].setPosition(pos + INCREMENT);
                    }
                    sleep(50);
                } else if (gamepad1.y && (sv_list[cur_sv_ix] != null)) {
                    double pos = sv_list[cur_sv_ix].getPosition();
                    if (pos >= INCREMENT) {
                        sv_list[cur_sv_ix].setPosition(pos - INCREMENT);
                    }
                    sleep(50);
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
            telemetry.addData("0.1 cvservo =", "%2.1f", cs_val);
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
            if (robot.jewel.use_color_sensor) {
                double r_d = robot.jewel.calcDelta(false);
                telemetry.addData("5.1 Color delta", "r_d=%1.0f", r_d);
            }
            show_telemetry();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            // robot.waitForTick(40);
        }
    }

}


