package org.firstinspires.ftc.teamcode;

/**
 * Created by carlw on 11/21/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SwervePosTest", group="SwerveDrive")
public class ServoPosTest extends SwerveUtilLOP {
    /* Declare OpMode members. */
    static final double INCREMENT = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        boolean serve_tune_up = true; // enable servo tune up
        robot.use_swerve = true;
        robot.use_imu = true;
        robot.use_Vuforia = false;
        robot.use_color_sensor = false;
        robot.use_arm = true;
        robot.use_glyph_grabber = false;
        robot.use_relic_grabber = false;
        robot.use_test_servo = false;
        robot.use_test_motor = false;

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
                robot.sv_glyph_grabber_bottom,
                robot.sv_glyph_grabber_top,
                robot.sv_relic_grabber,
                robot.sv_relic_arm
        };

        num_servos = sv_list.length;
        while (cur_sv_ix < num_servos && sv_list[cur_sv_ix] == null) {
            cur_sv_ix++;
        }
        if (cur_sv_ix == num_servos) cur_sv_ix = 0; // no servo available

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                robot.isTestingFL = true;
                robot.isTestingFR = false;
                robot.isTestingBL = false;
                robot.isTestingBR = false;

            }

            if (gamepad1.dpad_down) {
                robot.isTestingFL = false;
                robot.isTestingFR = false;
                robot.isTestingBL = false;
                robot.isTestingBR = true;

            }

            if (gamepad1.dpad_left) {
                robot.isTestingFL = false;
                robot.isTestingFR = false;
                robot.isTestingBL = true;
                robot.isTestingBR = false;

            }

            if (gamepad1.dpad_right) {
                robot.isTestingFL = false;
                robot.isTestingFR = true;
                robot.isTestingBL = false;
                robot.isTestingBR = false;

            }

            if (gamepad1.left_bumper) {
                sleep(100);
                test_swerve_servo(true);
            }

            if (gamepad1.right_bumper) {
                sleep(100);
                test_swerve_servo(false);
            }

            if(gamepad1.y){
                while(gamepad1.y) {
                    test_swerve_motor(1, true);
                }
                stop_chassis();
            }
            if(gamepad1.a){
                while(gamepad1.a) {
                    test_swerve_motor(1, false);
                }
                stop_chassis();
            }

            if(gamepad1.x){
                change_swerve_pos(SwerveDriveHardware.CarMode.STRAIGHT);
                sleep(100);
                TurnLeftD(0.4, 90);
            }

            if(gamepad1.b){
                change_swerve_pos(SwerveDriveHardware.CarMode.STRAIGHT);
                sleep(100);
                TurnRightD(0.4, 90);
            }

            telemetry.addData("3. W-sv angle FL/FR/BL/BR =", "%.3f/%.3f/%.3f/%.3f",
                    robot.servoPosFL, robot.servoPosFR, robot.servoPosBL, robot.servoPosBR);
            telemetry.addData("4.1 IMU Heading = ", "%.2f", imu_heading());
            telemetry.update();
        }
    }
}
