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

            if (robot.use_swerve) {

                if (robot.isCarMode) {
                    if(gamepad1.right_stick_x > 0.1){
                        robot.isSnakingLeft = false;
                    }
                    else{
                        robot.isSnakingLeft = true;
                    }
                    if(Math.abs(gamepad1.right_stick_x) < 0.2){
                        robot.enoughToSnake = false;
                    }
                    else{
                        robot.enoughToSnake = true;
                    }

                    if(Math.abs(gamepad1.right_stick_x) > 0.8){
                        robot.r_Value = robot.MAX_TURNING_RADIUS - (/*Hypothetical limiter here*/(robot.MAX_TURNING_RADIUS - robot.MIN_TURNING_RADIUS));
                    }
                    else{
                        robot.r_Value = robot.MAX_TURNING_RADIUS - (Math.abs(gamepad1.right_stick_x) * (robot.MAX_TURNING_RADIUS - robot.MIN_TURNING_RADIUS));
                    }

                    robot.thetaOneCalc = (Math.atan((0.5 * robot.WIDTH_BETWEEN_WHEELS) / ((robot.r_Value) - (0.5 * robot.LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 1
                    robot.thetaTwoCalc = (Math.atan((0.5 * robot.WIDTH_BETWEEN_WHEELS) / ((robot.r_Value) + (0.5 * robot.LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 2

                    if(robot.enoughToSnake) {
                        if(robot.isSnakingLeft) {
                            robot.leftServoAngle = 1 - (robot.thetaOneCalc);
                            robot.rightServoAngle = 1 - (robot.thetaTwoCalc);
                        }
                        else{
                            robot.leftServoAngle = robot.thetaTwoCalc;
                            robot.rightServoAngle = robot.thetaOneCalc;
                        }
                        robot.servoPosFL = 1 - (robot.leftServoAngle);
                        robot.servoPosFR = 1 - (robot.rightServoAngle);
                        robot.servoPosBL = robot.leftServoAngle;
                        robot.servoPosBR = robot.rightServoAngle;
                        robot.servoFrontLeft.setPosition(robot.servoPosFL);
                        robot.servoFrontRight.setPosition(robot.servoPosFR);
                        robot.servoBackLeft.setPosition(robot.servoPosBL);
                        robot.servoBackRight.setPosition(robot.servoPosBR);
                    }
                    else{
                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);
                    }

                    if(gamepad1.left_trigger > 0.1){
                        robot.isTurn = true;
                        robot.hasTeleTurnLeft = true;

                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_TURN_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_TURN_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_TURN_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_TURN_POSITION);

                        robot.motorFrontLeft.setPower(-gamepad1.left_trigger);
                        robot.motorFrontRight.setPower(gamepad1.left_trigger);
                        robot.motorBackLeft.setPower(-gamepad1.left_trigger);
                        robot.motorBackRight.setPower(gamepad1.left_trigger);
                    }
                    else if(robot.hasTeleTurnLeft){
                        robot.isTurn = false;
                        robot.hasTeleTurnLeft = false;

                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);

                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }

                    if(gamepad1.right_trigger > 0.1){
                        robot.isTurn = true;
                        robot.hasTeleTurnRight = true;

                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_TURN_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_TURN_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_TURN_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_TURN_POSITION);

                        robot.motorFrontLeft.setPower(gamepad1.right_trigger);
                        robot.motorFrontRight.setPower(-gamepad1.right_trigger);
                        robot.motorBackLeft.setPower(gamepad1.right_trigger);
                        robot.motorBackRight.setPower(-gamepad1.right_trigger);
                    }
                    else if(robot.hasTeleTurnRight){
                        robot.isTurn = false;
                        robot.hasTeleTurnRight = false;

                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);

                        robot.motorFrontLeft.setPower(0);
                        robot.motorFrontRight.setPower(0);
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }
                }
                else {
                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_TURN_POSITION);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_TURN_POSITION);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_TURN_POSITION);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_TURN_POSITION);

                        robot.motorFrontLeft.setPower(-gamepad1.left_trigger);
                        robot.motorFrontRight.setPower(gamepad1.left_trigger);
                        robot.motorBackLeft.setPower(-gamepad1.left_trigger);
                        robot.motorBackRight.setPower(gamepad1.left_trigger);
                    }
                } else {
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
                            robot.servoFrontLeft.setPosition(robot.SERVO_FL_TURN_POSITION);
                            robot.servoFrontRight.setPosition(robot.SERVO_FR_TURN_POSITION);
                            robot.servoBackLeft.setPosition(robot.SERVO_BL_TURN_POSITION);
                            robot.servoBackRight.setPosition(robot.SERVO_BR_TURN_POSITION);

                            robot.servoPosFL = robot.SERVO_FL_TURN_POSITION;
                            robot.servoPosFR = robot.SERVO_FR_TURN_POSITION;
                            robot.servoPosBL = robot.SERVO_BL_TURN_POSITION;
                            robot.servoPosBR = robot.SERVO_BR_TURN_POSITION;

                            robot.isTurn = true;
                            robot.isForward = false;
                        } else if (robot.isTurn) {
                            robot.servoFrontLeft.setPosition(robot.SERVO_FL_STRAFE_POSITION);
                            robot.servoFrontRight.setPosition(robot.SERVO_FR_STRAFE_POSITION);
                            robot.servoBackLeft.setPosition(robot.SERVO_BL_STRAFE_POSITION);
                            robot.servoBackRight.setPosition(robot.SERVO_BR_STRAFE_POSITION);

                            robot.servoPosFL = robot.SERVO_FL_STRAFE_POSITION;
                            robot.servoPosFR = robot.SERVO_FR_STRAFE_POSITION;
                            robot.servoPosBL = robot.SERVO_BL_STRAFE_POSITION;
                            robot.servoPosBR = robot.SERVO_BR_STRAFE_POSITION;

                            robot.isTurn = false;
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
                        sleep(400);
                    }

                    if(gamepad1.b) {
                        StraightIn(-0.3, 50);
                    }
                    if(gamepad1.left_bumper) {
                        sleep(100);
                        TurnLeftD(0.5, 90.0);
                    }

                    if(gamepad1.right_bumper) {
                        sleep(100);
                        TurnRightD(0.5, 90.0);
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
                if (Math.abs(robot.servoPosFL) < 0) {
                    robot.servoPosFL = 0.0;
                }
                if (Math.abs(robot.servoPosFR) < 0) {
                    robot.servoPosFR = 0.0;
                }
                if (Math.abs(robot.servoPosBL) < 0) {
                    robot.servoPosBL = 0.0;
                }
                if (Math.abs(robot.servoPosBR) < 0) {
                    robot.servoPosBR = 0.0;
                }

                if(robot.isCarMode) {
                    robot.insideWheelsMod = gamepad1.left_stick_y * (Math.pow((Math.pow(0.5 * robot.LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) - robot.WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                            (robot.r_Value);
                    robot.outsideWheelsMod = gamepad1.left_stick_y * (Math.pow((Math.pow(0.5 * robot.LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) + robot.WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                            (robot.r_Value);
                    if (robot.enoughToSnake) {
                        if (robot.isSnakingLeft) {
                            robot.motorPowerLeft = robot.insideWheelsMod;
                            robot.motorPowerRight = robot.outsideWheelsMod;
                        } else {
                            robot.motorPowerLeft = robot.outsideWheelsMod;
                            robot.motorPowerRight = robot.insideWheelsMod;
                        }
                    } else {
                        robot.motorPowerLeft = gamepad1.left_stick_y;
                        robot.motorPowerRight = gamepad1.left_stick_y;
                    }
                } else if (robot.isTurn) {
                    robot.motorPowerTurn = gamepad1.right_stick_x;
                    robot.motorFrontLeft.setPower(-robot.motorPowerTurn);
                    robot.motorFrontRight.setPower(robot.motorPowerTurn);
                    robot.motorBackLeft.setPower(-robot.motorPowerTurn);
                    robot.motorBackRight.setPower(robot.motorPowerTurn);
                } else {
                    robot.motorPowerLeft = gamepad1.left_stick_y;
                    robot.motorPowerRight = gamepad1.right_stick_y;
                    if (robot.isForward) {
                        robot.motorFrontLeft.setPower(robot.motorPowerLeft);
                        robot.motorFrontRight.setPower(robot.motorPowerRight);
                        robot.motorBackLeft.setPower(robot.motorPowerLeft);
                        robot.motorBackRight.setPower(robot.motorPowerRight);
                    } else {
                        robot.motorFrontLeft.setPower(-robot.motorPowerRight);
                        robot.motorFrontRight.setPower(robot.motorPowerRight);
                        robot.motorBackLeft.setPower(robot.motorPowerLeft);
                        robot.motorBackRight.setPower(-robot.motorPowerLeft);
                    }
                }
            } // end use_swerve
            show_telemetry();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
