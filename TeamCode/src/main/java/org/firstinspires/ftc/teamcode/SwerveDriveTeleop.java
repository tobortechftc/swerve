package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name="SwerveDrive: Teleop", group="SwerveDrive")
public class SwerveDriveTeleop extends LinearOpMode {
    
    /* Declare OpMode members. */
    SwerveDriveHardware robot           = new SwerveDriveHardware();

    @Override
    public void runOpMode() {
        double motorPower;
        double servoPos;
        double max;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // The left stick up/down moves the wheels forward and backwards, while the right stick left/right should rotate the wheels
            motorPower = -gamepad1.left_stick_y;
            servoPos = -gamepad1.right_stick_x;

            // When the left bumper is pressed, it moves the servos to a -45 degree bearing.
            // When the right bumper is pressed, it moves the servos to a 45 degree bearing.
            if (gamepad1.left_bumper) {
                servoPos = -.5;
            }
            if (gamepad1.right_bumper) {
                servoPos = .5;
            }

            // Possible idea where holding the left stick farther from the center makes it turn the servo farther. Not completed.
//            while (-gamepad1.left_stick_y > .10 && -gamepad1.left_stick_y < -.10) {
//                servoPos = -gamepad1.left_stick_y * .2;
//            }

            // Normalize the values so neither exceed +/- 1.0

            max = Math.max(Math.abs(motorPower), Math.abs(servoPos));
            if (max > 1.0)
            {
                motorPower /= max;
                servoPos /= max;
            }

            robot.motorFrontLeft.setPower(motorPower);
            robot.motorFrontRight.setPower(motorPower);
            robot.motorBackLeft.setPower(motorPower);
            robot.motorBackRight.setPower(motorPower);

            robot.servoFrontLeft.setPower(servoPos);
            robot.servoFrontRight.setPower(servoPos);
            robot.servoBackLeft.setPower(servoPos);
            robot.servoBackRight.setPower(servoPos);


            telemetry.addData("power level  =", "%.2f", motorPower);
            telemetry.addData("rotation angle =", "%.2f", servoPos);
            telemetry.update();


            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
