package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name="SwerveDrive: Teleop", group="SwerveDrive")
public class SwerveDriveTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    SwerveDriveHardware robot           = new SwerveDriveHardware();

    @Override
    public void runOpMode() {
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
            robot.motorPowerLeft = -gamepad1.left_stick_y;
            robot.motorPowerRight = -gamepad1.right_stick_y;


            if (gamepad1.x) {
                if (robot.isForward) {
                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_STRAFE_POSITION);
                    robot.servoFrontRight.setPosition(robot.SERVO_FR_STRAFE_POSITION);
                    robot.servoBackLeft.setPosition(robot.SERVO_BL_STRAFE_POSITION);
                    robot.servoBackRight.setPosition(robot.SERVO_BR_STRAFE_POSITION);
                    robot.isForward = false;
                }
                else  {
                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
                    robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                    robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);
                    robot.isForward = true;
                }
                sleep(100);
            }

            if (gamepad1.b){
                if(robot.isTurn){
                    if(robot.isForward){
                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);
                    }
                    else{
                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_STRAFE_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_STRAFE_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_STRAFE_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_STRAFE_POSITION);
                    }
                }
                else{
                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_TURN_POSITION);
                    robot.servoFrontRight.setPosition(robot.SERVO_FR_TURN_POSITION);
                    robot.servoBackLeft.setPosition(robot.SERVO_BL_TURN_POSITION);
                    robot.servoBackRight.setPosition(robot.SERVO_BR_TURN_POSITION);
                }
            }

            // Possible idea where holding the left stick farther from the center makes it turn the servo farther. Not completed.
//            while (-gamepad1.left_stick_y > .10 && -gamepad1.left_stick_y < -.10) {
//                servoPos = -gamepad1.left_stick_y * .2;
//            }

            // Normalize the values so neither exceed +/- 1.0

            if (Math.abs(robot.motorPowerLeft) > 1) {
                robot.motorPowerLeft = 1.0;
            }
            if (Math.abs(robot.motorPowerRight) > 1) {
                robot.motorPowerRight = 1.0;
            }
            if (Math.abs(robot.servoPosFL) > 1) {
                robot.servoPosFL = 1.0;
            }
            if (Math.abs(robot.servoPosFR) > 1) {
                robot.servoPosFR = 1.0;
            }
            if (Math.abs(robot.servoPosBL) > 1) {
                robot.servoPosBL = 1.0;
            }
            if (Math.abs(robot.servoPosBR) > 1) {
                robot.servoPosBR = 1.0;
            }
            if (robot.isForward) {
                robot.motorFrontLeft.setPower(robot.motorPowerLeft);
                robot.motorFrontRight.setPower(robot.motorPowerRight);
                robot.motorBackLeft.setPower(robot.motorPowerLeft);
                robot.motorBackRight.setPower(robot.motorPowerRight);
            }
            else {
                robot.motorFrontLeft.setPower(-robot.motorPowerLeft);
                robot.motorFrontRight.setPower(robot.motorPowerRight);
                robot.motorBackLeft.setPower(robot.motorPowerLeft);
                robot.motorBackRight.setPower(-robot.motorPowerRight);
            }



            telemetry.addData("power level left =", "%.2f", robot.motorPowerLeft);
            telemetry.addData("power level Right =", "%.2f", robot.motorPowerRight);
            telemetry.addData("rotation angle front =", "%.2f", robot.servoPosFL, robot.servoPosFR);
            telemetry.addData("rotation angle back =", "%.2f", robot.servoPosBL, robot.servoPosBR);
            telemetry.update();


            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
