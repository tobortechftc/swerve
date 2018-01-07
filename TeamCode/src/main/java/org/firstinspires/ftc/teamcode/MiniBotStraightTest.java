
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="MiniBot: StraightTest", group="MiniBot")
public class MiniBotStraightTest extends SwerveUtilLOP {

    /* Declare OpMode members. */
    SwerveDriveHardware robot           = new SwerveDriveHardware();   // Use a Pushbot's hardware
    static double joy_threshold = 0.03;                                                          // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() {

        robot.use_swerve = false;
        robot.use_imu = false;
        robot.use_encoder = false;
        robot.use_minibot = true;
        robot.use_range_sensor = false;
        robot.use_color_sensor = false;
        robot.use_Vuforia = false;

        double left=0;
        double right=0;
        double max;
        double prev_right_y = 0;
        double prev_left_y = 0;
        double speedscaleLeft = 0.5;
        double speedscaleRight = 0.5;
        int left_event_counter =0;
        int right_event_counter =0;
        int loop_count = 0;
        int delay_scale = 100;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        double power_steps [] = {0.0, 0.5, 0.7, 0.85, 0.95, 1.0};
        int n_steps = 6;
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // if (Math.abs(prev_left_y - gamepad1.left_stick_y)>0) {
            if (left==0.0 && Math.abs(gamepad1.left_stick_y)>joy_threshold) {
                left_event_counter = 1;
            } else if (left>0.1 && Math.abs(gamepad1.left_stick_y)<joy_threshold) {
                left_event_counter = 5;
            } else {
                if (Math.abs(gamepad1.left_stick_y)>joy_threshold) {
                    if ((left_event_counter < n_steps-1) && (loop_count%delay_scale==4)) {
                        left_event_counter++;
                    }
                } else {
                    if ((left_event_counter > 0)&& (loop_count%delay_scale==4)) {
                        left_event_counter--;
                    }
                }
            }
            // if (Math.abs(prev_right_y - gamepad1.right_stick_y)>0) {
            if (right==0.0 && Math.abs(gamepad1.right_stick_y)>joy_threshold) {
                right_event_counter = 1;
            } else if (right>0.1 && Math.abs(gamepad1.right_stick_y)<joy_threshold) {
                right_event_counter = 5;
            } else {
                if (Math.abs(gamepad1.right_stick_y)>joy_threshold) {
                    if ((right_event_counter < n_steps-1) && (loop_count%delay_scale==4)) {
                        right_event_counter++;
                    }
                } else {
                    if ((right_event_counter > 0)&& (loop_count%delay_scale==4)) {
                        right_event_counter--;
                    }
                }
            }
            //left_event_counter = right_event_counter = 5;
            left = -gamepad1.left_stick_y*power_steps[left_event_counter];
            right = -gamepad1.right_stick_y*power_steps[right_event_counter];
            if ((loop_count%5)==1) {
                prev_right_y = -gamepad1.right_stick_y;
                prev_left_y = -gamepad1.left_stick_y;
            }

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            left *= speedscaleRight;
            right *= speedscaleLeft;
            robot.motorFrontLeft.setPower(left);
            robot.motorFrontRight.setPower(right);
            if (gamepad1.left_bumper && (speedscaleLeft < 1.0)) {
                speedscaleLeft += 0.05;
                //robot.waitForTick(40);
            }
            else if (gamepad1.left_trigger >= .5 && (speedscaleLeft > 0.2)) {
                speedscaleLeft -= 0.05;
                //robot.waitForTick(40);
            }

            if (gamepad1.right_bumper && (speedscaleRight < 1.0)) {
                speedscaleLeft += 0.05;
                //robot.waitForTick(40);
            }
            else if (gamepad1.right_trigger >= .5 && (speedscaleRight > 0.2)) {
                speedscaleLeft -= 0.05;
                //robot.waitForTick(40);
            }


            telemetry.addData("left/right motor  =", "%.2f/%.2f", left,right);
            // telemetry.addData("prev left/right y  =", "%.2f/%.2f", prev_left_y,prev_right_y);
            telemetry.addData("left/right counter =", "%d/%d", left_event_counter, right_event_counter);
            telemetry.addData("left speed scale =", "%.2f", speedscaleLeft);
            telemetry.addData("right speed scale =", "%.2f", speedscaleRight);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            loop_count++;
        }
    }
}
