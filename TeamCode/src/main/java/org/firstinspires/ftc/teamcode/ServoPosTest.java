package org.firstinspires.ftc.teamcode;

/**
 * Created by carlw on 11/21/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestSwervePos", group="Test")
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
            if (gamepad1.left_trigger>0.1) {
                if (robot.cur_mode==SwerveDriveHardware.CarMode.CAR ||
                        robot.cur_mode==SwerveDriveHardware.CarMode.STRAIGHT) {
                    change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
                    sleep(100);
                } else {
                    change_swerve_pos(SwerveDriveHardware.CarMode.STRAIGHT);
                }
                sleep(200);
            } else if(gamepad1.y){
                while(gamepad1.y) {
                    test_swerve_motor(1, true);
                }
                stop_chassis();
            } else if(gamepad1.a){
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
            if (Math.abs(gamepad1.left_stick_y)>0.1) {
                if (gamepad1.left_stick_y>0.1) {
                    robot.SERVO_90_DEGREE = robot.SERVO_90_DEGREE - 0.001;
                }
                else if (gamepad1.left_stick_y<-0.1) {
                    robot.SERVO_90_DEGREE = robot.SERVO_90_DEGREE + 0.001;
                }
                robot.SERVO_FL_STRAFE_POSITION = robot.SERVO_FL_FORWARD_POSITION + robot.SERVO_90_DEGREE;
                robot.SERVO_FR_STRAFE_POSITION = robot.SERVO_FR_FORWARD_POSITION - robot.SERVO_90_DEGREE;
                robot.SERVO_BL_STRAFE_POSITION = robot.SERVO_BL_FORWARD_POSITION + robot.SERVO_90_DEGREE;
                robot.SERVO_BR_STRAFE_POSITION = robot.SERVO_BR_FORWARD_POSITION - robot.SERVO_90_DEGREE;

                change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
            }

            if (robot.isTestingBL) {
                telemetry.addData("2. Testing BL","");
            } else if (robot.isTestingBR) {
                telemetry.addData("2. Testing BR","");
            } else if (robot.isTestingFL) {
                    telemetry.addData("2. Testing FL","");
            } else if (robot.isTestingFR) {
                    telemetry.addData("2. Testing FR","");
            }
            telemetry.addData("1 Ch mode (90%) = ", "%s (%.3f)",robot.cur_mode.toString(),robot.SERVO_90_DEGREE);
            telemetry.addData("3. W-sv FL/FR/BL/BR=", "%.3f/%.3f/%.3f/%.3f",
                    robot.servoPosFL, robot.servoPosFR, robot.servoPosBL, robot.servoPosBR);
            telemetry.addData("4.1 IMU Heading = ", "%.2f", imu_heading());
            telemetry.update();
        }
    }
}
