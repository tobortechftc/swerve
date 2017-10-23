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

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // The left stick up/down moves the wheels forward and backwards, while the right stick left/right should rotate the wheels

            if(gamepad1.back && gamepad1.a){
                if(!(robot.cur_mode == SwerveDriveHardware.CarMode.CAR)){// If in any other mode, switch to snake
                    change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
                }
                else{ //Return from snake to previous drive mode
                    change_swerve_pos(robot.old_mode);
                }
                sleep(400);
            }

            if (robot.use_swerve) {

                if(robot.isTesting){ //Allow to test individual servo positions
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

                    if (gamepad1.left_trigger > 0.1) {

                        sleep(100);
                        TurnLeftD(0.5, 90.0);
                    }
                    if (gamepad1.right_trigger > 0.1) {
                        sleep(100);
                        TurnRightD(0.5, 90.0);
                    }

                    if (gamepad1.left_bumper) {
                        sleep(100);
                        test_swerve_servo(true);
                    }

                    if (gamepad1.right_bumper) {
                        sleep(100);
                        test_swerve_servo(false);
                    }

                    if (gamepad1.b) {
                        StraightIn(0.5, 50);
                    }
                    if (gamepad1.a){
                        StraightIn(-0.5, 50);
                    }
                    if(gamepad1.y){
                        while(gamepad1.y) {
                            test_swerve_motor(1, true);
                        }
                        stop_chassis();
                    }
                    if(gamepad1.x){
                        while(gamepad1.x) {
                            test_swerve_motor(1, false);
                        }
                        stop_chassis();
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
                    if (gamepad1.left_trigger > 0.1) {

                        change_swerve_pos(SwerveDriveHardware.CarMode.TURN);

                        sleep(200);

                        while (gamepad1.left_trigger > 0.1) {
                            robot.motorFrontLeft.setPower(0.3);
                            robot.motorFrontRight.setPower(-0.3);
                            robot.motorBackLeft.setPower(0.3);
                            robot.motorBackRight.setPower(-0.3);
                        }

                        change_swerve_pos(robot.old_mode);

                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }
                    if (gamepad1.right_trigger > 0.1) {

                        change_swerve_pos(SwerveDriveHardware.CarMode.TURN);

                        sleep(200);

                        while (gamepad1.right_trigger > 0.1) {
                            robot.motorFrontLeft.setPower(-0.3);
                            robot.motorFrontRight.setPower(0.3);
                            robot.motorBackLeft.setPower(-0.3);
                            robot.motorBackRight.setPower(0.3);
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
                }




                if (robot.cur_mode == SwerveDriveHardware.CarMode.CAR) { //If in snake drive, calculate and change servo angles

                    calc_snake(gamepad1.right_stick_x);

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

                set_swerve_power(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);

            } // end use_swerve
            show_telemetry();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
