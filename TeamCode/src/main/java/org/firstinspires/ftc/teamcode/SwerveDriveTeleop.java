package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name="SwerveDrive: Teleop", group="SwerveDrive")
public class SwerveDriveTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    SwerveDriveHardware robot           = new SwerveDriveHardware();

    @Override
    public void runOpMode() throws InterruptedException{
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

            if(gamepad1.back && gamepad1.a){
                if(!robot.isCarMode){
                    robot.isCarMode = true;
                }
                else{
                    robot.isCarMode = false;
                }
                sleep(400);
            }


            if(robot.isCarMode){

                if(gamepad1.x){
                    if (robot.isTrueCar){
                        robot.isTrueCar = false;
                    }
                    else{
                        robot.isTrueCar = true;
                    }
                }

                if(robot.isTrueCar) {
                    robot.servoPosFL = (gamepad1.right_stick_x / 3) + 0.5;
                    robot.servoPosFR = (gamepad1.right_stick_x / 3) + 0.5;
                    robot.servoFrontLeft.setPosition(robot.servoPosFL);
                    robot.servoFrontRight.setPosition(robot.servoPosFR);
                }
                else{
                    robot.servoPosFL = 1 - ((gamepad1.right_stick_x / 6) + 0.5);
                    robot.servoPosFR = 1 - ((gamepad1.right_stick_x / 6) + 0.5);
                    robot.servoPosBL = ((gamepad1.right_stick_x / 6) + 0.5);
                    robot.servoPosBR = ((gamepad1.right_stick_x / 6) + 0.5);
                    robot.servoFrontLeft.setPosition(robot.servoPosFL);
                    robot.servoFrontRight.setPosition(robot.servoPosFR);
                    robot.servoBackLeft.setPosition(robot.servoPosBL);
                    robot.servoBackRight.setPosition(robot.servoPosBR);
                }
            }
            else {
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

                if(gamepad1.left_bumper) {
                    robot.TurnLeftD(0.2, 90.0);
                }

                if(gamepad1.right_bumper){
                    robot.TurnRightD(0.2, 90.0);
                }

                if (gamepad1.left_trigger > 0.1) {

                    if (robot.isTestingFL) {
                        robot.servoFrontLeft.setPosition(robot.servoPosFL + 0.01);
                        robot.servoPosFL += 0.01;
                    } else if (robot.isTestingFR) {
                        robot.servoFrontRight.setPosition(robot.servoPosFR + 0.01);
                        robot.servoPosFR += 0.01;
                    } else if (robot.isTestingBL) {
                        robot.servoBackLeft.setPosition(robot.servoPosBL + 0.01);
                        robot.servoPosBL += 0.01;
                    } else if (robot.isTestingBR) {
                        robot.servoBackRight.setPosition(robot.servoPosBR + 0.01);
                        robot.servoPosBR += 0.01;
                    } else {

                        robot.servoFrontLeft.setPosition(robot.servoPosFL + 0.05);
                        robot.servoPosFL += 0.05;

                        robot.servoFrontRight.setPosition(robot.servoPosFR + 0.05);
                        robot.servoPosFR += 0.05;

                        robot.servoBackLeft.setPosition(robot.servoPosBL + 0.05);
                        robot.servoPosBL += 0.05;

                        robot.servoBackRight.setPosition(robot.servoPosBR + 0.05);
                        robot.servoPosBR += 0.05;
                    }
                }
                if (gamepad1.right_trigger > 0.1) {

                    if (robot.isTestingFL) {
                        robot.servoFrontLeft.setPosition(robot.servoPosFL - 0.01);
                        robot.servoPosFL -= 0.01;
                    } else if (robot.isTestingFR) {
                        robot.servoFrontRight.setPosition(robot.servoPosFR - 0.01);
                        robot.servoPosFR -= 0.01;
                    } else if (robot.isTestingBL) {
                        robot.servoBackLeft.setPosition(robot.servoPosBL - 0.01);
                        robot.servoPosBL -= 0.01;
                    } else if (robot.isTestingBR) {
                        robot.servoBackRight.setPosition(robot.servoPosBR - 0.01);
                        robot.servoPosBR -= 0.01;
                    } else {

                        robot.servoFrontLeft.setPosition(robot.servoPosFL - 0.05);
                        robot.servoPosFL -= 0.05;

                        robot.servoFrontRight.setPosition(robot.servoPosFR - 0.05);
                        robot.servoPosFR -= 0.05;

                        robot.servoBackLeft.setPosition(robot.servoPosBL - 0.05);
                        robot.servoPosBL -= 0.05;

                        robot.servoBackRight.setPosition(robot.servoPosBR - 0.05);
                        robot.servoPosBR -= 0.05;
                    }
                }

                if (gamepad1.a) {
                    robot.isTestingFL = false;
                    robot.isTestingFR = false;
                    robot.isTestingBL = false;
                    robot.isTestingBR = false;
                }

                if (gamepad1.y) {
                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
                    robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                    robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);

                    robot.servoPosFL = robot.SERVO_FL_FORWARD_POSITION;
                    robot.servoPosFR = robot.SERVO_FR_FORWARD_POSITION;
                    robot.servoPosBL = robot.SERVO_BL_FORWARD_POSITION;
                    robot.servoPosBR = robot.SERVO_BR_FORWARD_POSITION;

                    robot.isForward = true;
                    robot.isTurn = false;
                }


                if (gamepad1.x) {
                    if (robot.isForward) {
                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_STRAFE_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_STRAFE_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_STRAFE_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_STRAFE_POSITION);

                        robot.servoPosFL = robot.SERVO_FL_STRAFE_POSITION;
                        robot.servoPosFR = robot.SERVO_FR_STRAFE_POSITION;
                        robot.servoPosBL = robot.SERVO_BL_STRAFE_POSITION;
                        robot.servoPosBR = robot.SERVO_BR_STRAFE_POSITION;

                        robot.isForward = false;
                    } else {
                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);

                        robot.servoPosFL = robot.SERVO_FL_FORWARD_POSITION;
                        robot.servoPosFR = robot.SERVO_FR_FORWARD_POSITION;
                        robot.servoPosBL = robot.SERVO_BL_FORWARD_POSITION;
                        robot.servoPosBR = robot.SERVO_BR_FORWARD_POSITION;

                        robot.isForward = true;
                    }
                    robot.isTurn = false;
                    sleep(400);
                }
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
                    robot.isTurn = false;
                }
                else{
                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_TURN_POSITION);
                    robot.servoFrontRight.setPosition(robot.SERVO_FR_TURN_POSITION);
                    robot.servoBackLeft.setPosition(robot.SERVO_BL_TURN_POSITION);
                    robot.servoBackRight.setPosition(robot.SERVO_BR_TURN_POSITION);
                    robot.isTurn = true;
                }
                sleep(400);
            }

            // Possible idea where holding the left stick farther from the center makes it turn the servo farther. Not completed.
//            while (-gamepad1.left_stick_y > .10 && -gamepad1.left_stick_y < -.10) {
//                servoPos = -gamepad1.left_stick_y * .2;
//            }

            // Normalize the values so neither exceed 1.0 or 0.0

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
            if (Math.abs(robot.servoPosFL) < 0){
                robot.servoPosFL = 0.0;
            }
            if (Math.abs(robot.servoPosFR) < 0){
                robot.servoPosFR = 0.0;
            }
            if (Math.abs(robot.servoPosBL) < 0){
                robot.servoPosBL = 0.0;
            }
            if (Math.abs(robot.servoPosBR) < 0){
                robot.servoPosBR = 0.0;
            }

            if(robot.isCarMode){
                robot.motorPowerLeft = gamepad1.left_stick_y;
                robot.motorPowerRight = gamepad1.left_stick_y;

                robot.motorFrontLeft.setPower(robot.motorPowerLeft);
                robot.motorFrontRight.setPower(robot.motorPowerRight);
                robot.motorBackLeft.setPower(robot.motorPowerLeft);
                robot.motorBackRight.setPower(robot.motorPowerRight);

            }
            else if(robot.isTurn){
                robot.motorPowerTurn = gamepad1.right_stick_x;
                robot.motorFrontLeft.setPower(-robot.motorPowerTurn);
                robot.motorFrontRight.setPower(robot.motorPowerTurn);
                robot.motorBackLeft.setPower(-robot.motorPowerTurn);
                robot.motorBackRight.setPower(robot.motorPowerTurn);
            }
            else{
                robot.motorPowerLeft = gamepad1.left_stick_y;
                robot.motorPowerRight = gamepad1.right_stick_y;
                if (robot.isForward) {
                    robot.motorFrontLeft.setPower(robot.motorPowerLeft);
                    robot.motorFrontRight.setPower(robot.motorPowerRight);
                    robot.motorBackLeft.setPower(robot.motorPowerLeft);
                    robot.motorBackRight.setPower(robot.motorPowerRight);
                }
                else {
                    robot.motorFrontLeft.setPower(-robot.motorPowerRight);
                    robot.motorFrontRight.setPower(robot.motorPowerRight);
                    robot.motorBackLeft.setPower(robot.motorPowerLeft);
                    robot.motorBackRight.setPower(-robot.motorPowerLeft);
                }
            }



            telemetry.addData("power level left =", "%.2f", robot.motorPowerLeft);
            telemetry.addData("power level Right =", "%.2f", robot.motorPowerRight);
            telemetry.addData("rotation angle front left =", "%.2f", robot.servoPosFL);
            telemetry.addData("rotation angle front right =", "%.2f", robot.servoPosFR);
            telemetry.addData("rotation angle back left =", "%.2f", robot.servoPosBL);
            telemetry.addData("rotation angle back right =", "%.2f", robot.servoPosBR);
            if(robot.isCarMode){
                telemetry.addLine("Currently in: Car Mode");
            }
            else if(robot.isTurn){
                telemetry.addLine("Currently in: Quick Turn Mode");
            }
            else if(robot.isForward){
                telemetry.addLine("Currently in: Standard Mode");
            }
            else{
                telemetry.addLine("Currently in: Strafe Mode");
            }
            telemetry.update();


            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
