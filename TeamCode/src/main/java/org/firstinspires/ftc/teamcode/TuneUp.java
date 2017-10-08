package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Tune-up", group="SwerveDrive")
public class TuneUp extends SwerveUtilLOP {

    /* Declare OpMode members. */
    static final double INCREMENT   = 0.005;

    @Override
    public void runOpMode() throws InterruptedException{
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.use_swerve = false;
        robot.use_imu = false;
        robot.use_Vuforia = false;
        robot.use_color_sensor = true;
        robot.use_arm = true;
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("My name is Tobot.", "Need tune-up?");
        telemetry.update();

        int num_servos = 12;
        int cur_sv_ix = 0;
        boolean show_all = true;
        Servo [] sv_list = {
                robot.servoFrontLeft,
                robot.servoFrontRight,
                robot.servoBackLeft,
                robot.servoBackRight,
                robot.sv_shoulder,
                robot.sv_elbow,
                robot.sv_glyph_grabber_bottom,
                robot.sv_glyph_grabber_top,
                robot.sv_relic_grabber,
                robot.sv_relic_arm
        };

        num_servos = sv_list.length;
        while (sv_list[cur_sv_ix]==null && cur_sv_ix<num_servos) {
            cur_sv_ix ++;
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

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
                if (pos <= (1-INCREMENT)) {
                    robot.sv_shoulder.setPosition(pos + INCREMENT);
                }
            }
            if (gamepad2.dpad_right) {
                double pos = robot.sv_elbow.getPosition();
                if (pos <= (1-INCREMENT)) {
                    robot.sv_elbow.setPosition(pos + INCREMENT);
                }
            }
            if (gamepad2.dpad_left) {
                double pos = robot.sv_elbow.getPosition();
                if (pos >= INCREMENT) {
                    robot.sv_elbow.setPosition(pos - INCREMENT);
                }
            }
            if (gamepad2.a) {
                robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
            }
            if (gamepad2.y) {
                robot.sv_elbow.setPosition(robot.SV_ELBOW_UP);
            }
            if (gamepad2.x) {
                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_LEFT);
            }
            if (gamepad2.b) {
                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_RIGHT);
            }
            if (gamepad2.left_bumper) {
                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_DOWN);
            }


            if (robot.use_swerve) { // swerve related control

            } // end use_swerve
            if (gamepad1.back && gamepad1.a) {
               show_all = !show_all;
                sleep(50);
            } else if (gamepad1.a && (sv_list[cur_sv_ix]!=null)) {
                double pos = sv_list[cur_sv_ix].getPosition();
                if (pos <= (1-INCREMENT)) {
                    sv_list[cur_sv_ix].setPosition(pos + INCREMENT);
                }
            } else if (gamepad1.y && (sv_list[cur_sv_ix]!=null)) {
                double pos = sv_list[cur_sv_ix].getPosition();
                if (pos >= INCREMENT) {
                    sv_list[cur_sv_ix].setPosition(pos - INCREMENT);
                }
            }
            if (gamepad1.x) {
                cur_sv_ix --;
                if (cur_sv_ix<0) cur_sv_ix = num_servos - 1;
                while (sv_list[cur_sv_ix]==null && cur_sv_ix>0) {
                    cur_sv_ix --;
                }
                sleep(50);
            }
            if (gamepad1.b) {
                cur_sv_ix ++;
                if (cur_sv_ix>=num_servos) cur_sv_ix = 0;
                while (sv_list[cur_sv_ix]==null && cur_sv_ix<num_servos) {
                    cur_sv_ix ++;
                }
                sleep(50);
            }
            telemetry.addData("0. Gamepad1:", "X/B:servo select, Y/A:+/-%4.3f",INCREMENT);
            if (show_all) {
                for (int i=0; i<num_servos; i++) {
                    if (sv_list[i]!=null) {
                        telemetry.addData("6.", "%d. Servo %s = %5.4f", i, sv_list[i].getDeviceName(), sv_list[i].getPosition());
                    }
                }
            }
            else if (sv_list[cur_sv_ix]!=null) {
                telemetry.addData("7. Tune-up servo", "%s (ix=%d)", cur_sv_ix, sv_list[cur_sv_ix].getDeviceName());
                telemetry.addData("7.1. servo value = ", "%5.4f", sv_list[cur_sv_ix].getPosition());
            } else {
                telemetry.addLine("7. No active servo to tune-up.");
            }
            show_telemetry();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
