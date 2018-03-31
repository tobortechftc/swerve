/*
    Drives forward with range sensor pointing behind it. Waits a short time. Continues until range sensor detects
    enough space between the balance board and itself, then it stops.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Nick on 10/20/2017.
 */

@TeleOp(name = "Mini-TeleOp", group = "Swerve")
@Disabled
public class MiniTeleOp extends SwerveUtilLOP {
    static double JOY_THRESHOLD = 0.03;
    static final double INCREMENT   = 1.0/256.0;

    @Override
    public void runOpMode() throws InterruptedException {
        double left=0;
        double right=0;
        double max;
        double prev_right_y=0;
        double prev_left_y=0;
        double speedscale = 0.7;
        int left_event_counter =0;
        int right_event_counter =0;
        int loop_count = 0;
        int delay_scale = 100;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        //double power_steps [] = {0.0, 0.5, 0.7, 0.85, 0.95, 1.0};
        double power_steps [] = {0.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        int n_steps = 6;

        robot.use_Vuforia = false;
        robot.use_imu = false;
        robot.use_encoder = true;
        robot.use_minibot = true;
        robot.use_imu = true;
        robot.use_range_sensor = true;
        robot.use_color_sensor = false;
        robot.use_test_servo = false;
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if (left == 0.0 && Math.abs(gamepad1.left_stick_y) > JOY_THRESHOLD) {
                left_event_counter = 1;
            } else if (left > 0.1 && Math.abs(gamepad1.left_stick_y) < JOY_THRESHOLD) {
                left_event_counter = 5;
            } else {
                if (Math.abs(gamepad1.left_stick_y) > JOY_THRESHOLD) {
                    if ((left_event_counter < n_steps - 1) && (loop_count % delay_scale == 4)) {
                        left_event_counter++;
                    }
                } else {
                    if ((left_event_counter > 0) && (loop_count % delay_scale == 4)) {
                        left_event_counter--;
                    }
                }
            }
            // if (Math.abs(prev_right_y - gamepad1.right_stick_y)>0) {
            if (right == 0.0 && Math.abs(gamepad1.right_stick_y) > JOY_THRESHOLD) {
                right_event_counter = 1;
            } else if (right > 0.1 && Math.abs(gamepad1.right_stick_y) < JOY_THRESHOLD) {
                right_event_counter = 5;
            } else {
                if (Math.abs(gamepad1.right_stick_y) > JOY_THRESHOLD) {
                    if ((right_event_counter < n_steps - 1) && (loop_count % delay_scale == 4)) {
                        right_event_counter++;
                    }
                } else {
                    if ((right_event_counter > 0) && (loop_count % delay_scale == 4)) {
                        right_event_counter--;
                    }
                }
            }
            //left_event_counter = right_event_counter = 5;
            left = -gamepad1.left_stick_y * power_steps[left_event_counter];
            right = -gamepad1.right_stick_y * power_steps[right_event_counter];
            if ((loop_count % 5) == 1) {
                prev_right_y = -gamepad1.right_stick_y;
                prev_left_y = -gamepad1.left_stick_y;
            }

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            left *= speedscale;
            right *= speedscale;
            driveTT(left, right);
            if (gamepad1.y && (speedscale < 1.0)) {
                speedscale += 0.05;
                //robot.waitForTick(40);
            } else if (gamepad1.a && (speedscale > 0.2)) {
                speedscale -= 0.05;
                //robot.waitForTick(40);
            }
            //if (gamepad1.a && (robot.sv_test != null)) {
            //    double pos = robot.sv_test.getPosition();
            //    if (pos <= (1 - INCREMENT)) {
            //        robot.sv_test.setPosition(pos + INCREMENT);
            //    } else robot.sv_test.setPosition(1);
            //    sleep(50);
            //} else if (gamepad1.y && (robot.sv_test != null)) {
            //    double pos = robot.sv_test.getPosition();
            //    if (pos >= INCREMENT) {
            //        robot.sv_test.setPosition(pos - INCREMENT);
             //   } else robot.sv_test.setPosition(0);
             //   sleep(50);
            //}
            telemetry.addData("left/right/scale of motors  =", "%.2f/%.2f/%.2f", left, right, speedscale);
            telemetry.addData("left/right counter =", "%d/%d", left_event_counter, right_event_counter);
            if (robot.use_test_servo) {
                // telemetry.addData("test servo =", " %3.4f", robot.sv_test.getPosition());
            }
            show_telemetry();
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            loop_count++;
        }
        stop_tobot();
    }
}