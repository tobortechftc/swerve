package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static java.lang.Thread.sleep;


@TeleOp(name="SwerveDrive: Teleop", group="SwerveDrive")
@Disabled
public class SwerveUtilLOP extends LinearOpMode {

    /* Declare OpMode members. */
    SwerveDriveHardware robot           = new SwerveDriveHardware();

    @Override
    public void runOpMode() throws InterruptedException{
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello! Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }

    public double imu_heading() {
        if (!robot.use_imu)
            return -1.0;

        robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return robot.angles.firstAngle;
    }

    public void driveTT(double lp, double rp) {
        if(!robot.fast_mode && robot.straight_mode) { // expect to go straight
            if (robot.use_imu) {
                double cur_heading = imu_heading();
                double heading_off_by = ((cur_heading - robot.target_heading) / 360);
                if(robot.use_swerve) {
                    if (cur_heading - robot.target_heading > 0.7) {
                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION - heading_off_by);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION - heading_off_by);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION + heading_off_by);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION + heading_off_by);
                    } else if (cur_heading - robot.target_heading < -0.7) {
                        robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION + heading_off_by);
                        robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION + heading_off_by);
                        robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION - heading_off_by);
                        robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION - heading_off_by);
                    }
                }
                else if(robot.use_minibot) {
                    if (cur_heading - robot.target_heading > 0.7) { // crook to left,  slow down right motor
                        if (rp > 0) rp *= 0.5;
                        else lp *= 0.5;
                    } else if (cur_heading - robot.target_heading < -0.7) { // crook to right, slow down left motor
                        if (lp > 0) lp *= 0.5;
                        else rp *= 0.5;
                    }
                }
            }
        }
        if(robot.isForward) {
            if (Math.abs(rp) > 0.3 && Math.abs(lp) > 0.3) {
                robot.motorFrontRight.setPower(rp * robot.DRIVE_RATIO_FR);
                robot.motorFrontLeft.setPower(lp * robot.DRIVE_RATIO_FL);
                if (!robot.use_minibot) {
                    robot.motorBackLeft.setPower(lp * robot.DRIVE_RATIO_BL);
                    robot.motorBackRight.setPower(rp * robot.DRIVE_RATIO_BR);
                }
            } else {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                if (!robot.use_minibot) {
                    robot.motorBackLeft.setPower(lp);
                    robot.motorBackRight.setPower(rp);
                }
            }
        }
        else{
            if (Math.abs(rp) > 0.3 && Math.abs(lp) > 0.3) {
                robot.motorFrontRight.setPower(rp * robot.DRIVE_RATIO_FR);
                robot.motorFrontLeft.setPower(-rp * robot.DRIVE_RATIO_FL);
                if (!robot.use_minibot) {
                    robot.motorBackRight.setPower(-lp * robot.DRIVE_RATIO_BR);
                    robot.motorBackLeft.setPower(lp * robot.DRIVE_RATIO_BL);
                }
            } else {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(-rp);
                if (!robot.use_minibot) {
                    robot.motorBackLeft.setPower(lp);
                    robot.motorBackRight.setPower(-lp);
                }
            }
        }
    }

    void stop_chassis() {
        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        if (!robot.use_minibot) {
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);
        }
    }

    void reset_chassis()  {
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (!robot.use_minibot) {
            robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        robot.leftCnt = 0;
        robot.rightCnt = 0;
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!robot.use_minibot) {
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void stop_auto() {
        if (robot.use_color_sensor) {
            robot.colorSensor.enableLed(false);
            robot.colorSensor.close();
            robot.use_color_sensor = false;
        }
        if (robot.use_Vuforia) {
            robot.relicTrackables.deactivate();
            robot.use_Vuforia = false;
        }
    }

    void stop_tobot() {
        if (robot.use_swerve||robot.use_minibot)
            stop_chassis();
        if (robot.use_color_sensor) {
            robot.colorSensor.enableLed(false);
            robot.colorSensor.close();
        }
        // stop all sensors
    }

    public void run_until_encoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        int leftTC1 = leftCnt;
        int rightTC1 = rightCnt;
        int leftTC2 = 0;
        int rightTC2 = 0;
        int leftTC0 = 0;
        int rightTC0 = 0;
        double initLeftPower = leftPower;
        double initRightPower = rightPower;
        if (leftPower > 0.4 && leftTC1 > 600 && !robot.fast_mode) {
            leftTC2 = 450;
            leftTC0 = 50;
            leftTC1 -= 500;
        }
        if (rightPower > 0.4 && rightTC1 > 600 && !robot.fast_mode) {
            rightTC2 = 450;
            rightTC0 = 50;
            rightTC1 -= 500;
        }
        if (rightTC0 > 0 || leftTC0 > 0) {
            driveTT(0.3, 0.3);
            while (!have_drive_encoders_reached(leftTC0, rightTC0) && (runtime.seconds()<7)) {
                driveTT(0.3, 0.3);
                // show_telemetry();
            }
        }
        driveTT(leftPower, rightPower);
        runtime.reset();

        while (!have_drive_encoders_reached(leftTC1, rightTC1) && (runtime.seconds() < 5)) {
            driveTT(leftPower, rightPower);
        }
        if (rightTC2 > 0 || leftTC2 > 0) {
            driveTT(0.2, 0.2);
            while (!have_drive_encoders_reached(leftTC2, rightTC2) && (runtime.seconds() < 7)) {
                driveTT(0.2, 0.2);
                // show_telemetry();
            }
        }
        stop_chassis();
    }

    boolean has_left_drive_encoder_reached(double p_count) {
        DcMotor mt = robot.motorFrontLeft;
        if (robot.leftPower < 0) {
            //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
            return (mt.getCurrentPosition() <= p_count);
        } else {
            //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
            return (mt.getCurrentPosition() >= p_count);
        }
    } // has_left_drive_encoder_reached

    boolean has_right_drive_encoder_reached(double p_count) {
        DcMotor mt = robot.motorFrontLeft;
        if (robot.rightPower < 0) {
            return (mt.getCurrentPosition() <= p_count);
        } else {
            return (mt.getCurrentPosition() >= p_count);
        }

    } // has_right_drive_encoder_reached

    boolean have_drive_encoders_reached(double p_left_count, double p_right_count) {
        DcMotor mt = robot.motorFrontLeft;
        boolean l_return = false;
        if (has_left_drive_encoder_reached(p_left_count) && has_right_drive_encoder_reached(p_right_count)) {
            l_return = true;
        } else if (has_left_drive_encoder_reached(p_left_count)) { // shift target encoder value from right to left
            double diff = Math.abs(p_right_count - mt.getCurrentPosition()) / 2;
            if (robot.leftPower < 0) {
                robot.leftCnt -= diff;
            } else {
                robot.leftCnt += diff;
            }
            if (robot.rightPower < 0) {
                robot.rightCnt += diff;
            } else {
                robot.rightCnt -= diff;
            }
        } else if (has_right_drive_encoder_reached(p_right_count)) { // shift target encoder value from left to right
            double diff = Math.abs(p_left_count - mt.getCurrentPosition()) / 2;
            if (robot.rightPower < 0) {
                robot.rightCnt -= diff;
            } else {
                robot.rightCnt += diff;
            }
            if (robot.leftPower < 0) {
                robot.leftCnt += diff;
            } else {
                robot.leftCnt -= diff;
            }
        }
        return l_return;
    } // have_encoders_reached

    void StraightR(double power, double n_rotations) throws InterruptedException {
        DcMotor mt = robot.motorFrontLeft;
        robot.straight_mode = true;
        reset_chassis();
        // set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = mt.getCurrentPosition();
        int rightEncode = mt.getCurrentPosition();
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        robot.leftCnt = (int) (robot.ONE_ROTATION * n_rotations);
        robot.rightCnt = (int) (robot.ONE_ROTATION * n_rotations);
        robot.leftPower = robot.rightPower = (float) power;
        if (power < 0) { // move backward
            robot.leftCnt = leftEncode - robot.leftCnt;
            robot.rightCnt = rightEncode - robot.rightCnt;
        } else {
            robot.leftCnt += leftEncode;
            robot.rightCnt += rightEncode;
        }
        run_until_encoder(robot.leftCnt, robot.leftPower, robot.rightCnt, robot.rightPower);
        robot.straight_mode = false;
        robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
        robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
        if (!robot.use_minibot) {
            robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);
        }
        if (!robot.fast_mode)
            sleep(135);
    }

    void StraightIn(double power, double in) throws InterruptedException {
        if (robot.use_imu) {
            robot.target_heading = imu_heading();
        }
        if (robot.use_encoder) {
            double numberR = in / robot.INCHES_PER_ROTATION;
            StraightR(-power, numberR);
        } else { // using timer
            double in_per_ms = 0.014 * power / 0.8;
            if (in_per_ms < 0) in_per_ms *= -1.0;
            long msec = (long) (in / in_per_ms);
            if (msec < 100) msec = 100;
            if (msec > 6000) msec = 6000; // limit to 6 sec
            driveTT(power, power);
            sleep(msec);
            driveTT(0, 0);
        }
    }

    void TurnRightD(double power, double degree) throws InterruptedException {

        double adjust_degree_imu = robot.IMU_ROTATION_RATIO_R * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        ElapsedTime     runtime = new ElapsedTime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = robot.motorFrontLeft.getCurrentPosition();
        int rightEncode = robot.motorFrontRight.getCurrentPosition();
        robot.leftCnt = (int) (robot.ONE_ROTATION * robot.RROBOT * degree / 720.0);
        robot.rightCnt = (int) (-robot.ONE_ROTATION * robot.RROBOT * degree / 720.0);

        robot.leftPower = (float) -power;
        robot.rightPower = (float) power;

        robot.servoFrontLeft.setPosition(robot.SERVO_FL_TURN_POSITION);
        robot.servoFrontRight.setPosition(robot.SERVO_FR_TURN_POSITION);
        if (!robot.use_minibot) {
            robot.servoBackLeft.setPosition(robot.SERVO_BL_TURN_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_TURN_POSITION);
        }

        sleep(300);

        robot.leftCnt += leftEncode;
        robot.rightCnt += rightEncode;

        //DbgLog.msg(String.format("imu Right Turn %.2f degree with %.2f power.", degree, power));
        if (robot.use_imu) {
            current_pos = imu_heading();
            robot.target_heading = current_pos - adjust_degree_imu;
            if (robot.target_heading <= -180) {
                robot.target_heading += 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos <= 0)) {
                current_pos += 360;
            }
            while ((current_pos >= robot.target_heading) && (runtime.seconds() < 4.0)) {
                current_pos = imu_heading();
                // DbgLog.msg(String.format("imu current/target heading = %.2f/%.2f",current_pos,target_heading));

                if (heading_cross_zero && (current_pos <= 0)) {
                    current_pos += 360;
                }
                driveTT(robot.leftPower, robot.rightPower);
            }
        } else {
            if (robot.use_encoder) {
                run_until_encoder(robot.leftCnt, robot.leftPower, robot.rightCnt, robot.rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(robot.leftPower, robot.rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
        sleep(300);
        if(robot.isForward){
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
            if (!robot.use_minibot) {
                robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);
            }
        }
        else{
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_STRAFE_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_STRAFE_POSITION);
            if (!robot.use_minibot) {
                robot.servoBackLeft.setPosition(robot.SERVO_BL_STRAFE_POSITION);
                robot.servoBackRight.setPosition(robot.SERVO_BR_STRAFE_POSITION);
            }
        }
        if (!robot.fast_mode)
            sleep(135);
    }

    void TurnLeftD(double power, double degree) throws InterruptedException {
        double adjust_degree_imu = robot.IMU_ROTATION_RATIO_L * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        ElapsedTime     runtime = new ElapsedTime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS)
        int leftEncode = robot.motorFrontLeft.getCurrentPosition();
        int rightEncode = robot.motorFrontRight.getCurrentPosition();
        robot.leftCnt = (int) (-robot.ONE_ROTATION * robot.RROBOT * degree / 720.0);
        robot.rightCnt = (int) (robot.ONE_ROTATION * robot.RROBOT * degree / 720.0);

        robot.leftPower = (float) power;
        robot.rightPower = (float) -power;

        robot.servoFrontLeft.setPosition(robot.SERVO_FL_TURN_POSITION);
        robot.servoFrontRight.setPosition(robot.SERVO_FR_TURN_POSITION);
        if (!robot.use_minibot) {
            robot.servoBackLeft.setPosition(robot.SERVO_BL_TURN_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_TURN_POSITION);
        }

        sleep(300);

        robot.leftCnt += leftEncode;
        robot.rightCnt += rightEncode;


        //DbgLog.msg(String.format("imu Left Turn %.2f degree with %.2f power.", degree, power));
        if (robot.use_imu) {
            current_pos = imu_heading();
            robot.target_heading = current_pos + adjust_degree_imu;
            if (robot.target_heading >= 180) {
                robot.target_heading -= 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos >= 0)) {
                current_pos -= 360;
            }
            //DbgLog.msg(String.format("imu Left Turn curr/target pos = %.2f/%.2f.", current_pos, target_heading));
            while ((current_pos <= robot.target_heading) && (runtime.seconds() < 5.0)) {
                current_pos = imu_heading();
                if (heading_cross_zero && (current_pos >= 0)) {
                    current_pos -= 360;
                }
                driveTT(robot.leftPower, robot.rightPower);
            }
        } else {
            if (robot.use_encoder) {
                run_until_encoder(robot.leftCnt, robot.leftPower, robot.rightCnt, robot.rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(robot.leftPower, robot.rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
        sleep(300);
        if(robot.isForward){
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
            if (!robot.use_minibot) {
                robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);
            }
        }
        else{
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_STRAFE_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_STRAFE_POSITION);
            if (!robot.use_minibot) {
                robot.servoBackLeft.setPosition(robot.SERVO_BL_STRAFE_POSITION);
                robot.servoBackRight.setPosition(robot.SERVO_BR_STRAFE_POSITION);
            }
        }
        if (!robot.fast_mode)
            sleep(135);
    }

    public int get_cryptobox_column() throws InterruptedException {

        int column = -1;
        if (!robot.use_Vuforia)
            return column;

        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < 5.0 && column == -1) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                column = getColumnIndex(vuMark);
            }
        }
        return column;
    }

    double calcDelta() throws InterruptedException {
        return (robot.colorSensor.blue() - robot.colorSensor.red());
    }

    void checkBallColor() throws InterruptedException {
        double d = calcDelta();
        if ( (d >= robot.BLUE_BALL_MIN) && (d <= robot.BLUE_BALL_MAX)) {
            robot.isBlueBall = true;
        } else {
            robot.isBlueBall = false;
        }
        if (d >= robot.RED_BALL_MIN && d <= robot.RED_BALL_MAX) {
            robot.isRedBall = true;
        } else {
            robot.isRedBall = false;
        }
    }

    int getColumnIndex(RelicRecoveryVuMark vuMark) throws InterruptedException {
        // return row index for Cryptograph
        // unknown : -1
        // left    :  0
        // center  :  1
        // right   :  2
        if (vuMark == RelicRecoveryVuMark.LEFT)
            return 0;
        else if (vuMark == RelicRecoveryVuMark.CENTER)
            return 1;
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
            return 2;

        return -1;
    }

    void arm_up() {
        robot.sv_elbow.setPosition(robot.SV_ELBOW_UP);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_UP);
    }

    void arm_down() {
        robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_DOWN);
    }
    void arm_left() {
        robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_LEFT);
    }

    void arm_right() {
        robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_RIGHT);
    }

    void show_telemetry() throws InterruptedException {
        telemetry.addData("1. Tobot/Imu/Vu =", "%s/%s/%s",
                (robot.use_swerve ?"on":"off"), (robot.use_imu?"on":"off"),
                (robot.use_Vuforia ?"on":"off"));
        telemetry.addData("2. W-pw Left/Right =", "%.2f/%.2f",
                robot.motorPowerLeft,robot.motorPowerRight);
        telemetry.addData("3. W-sv angle FL/FR/BL/BR =", "%.3f/%.3f/%.3f/%.3f",
                robot.servoPosFL, robot.servoPosFR, robot.servoPosBL, robot.servoPosBR);
        if (robot.use_imu) {
            telemetry.addData("4. IMU Heading = ", "%.2f", imu_heading());
        }
        if (robot.use_Vuforia) {
            telemetry.addData("5. Vuforia Column = ", "%d", get_cryptobox_column());
        }
        if (robot.use_swerve) {
            if (robot.isCarMode) {
                telemetry.addLine("6. Currently in: Car Mode");
            } else if (robot.isTurn) {
                telemetry.addLine("6. Currently in: Quick Turn Mode");
            } else if (robot.isForward) {
                telemetry.addLine("6. Currently in: Standard Mode");
            } else {
                telemetry.addLine("6. Currently in: Strafe Mode");
            }
            telemetry.addData("7. Encoder values FL/FR/BL/BR = ", "%d/%d/%d/%d",
                    robot.motorFrontLeft.getCurrentPosition(), robot.motorFrontRight.getCurrentPosition(), robot.motorBackLeft.getCurrentPosition(), robot.motorBackRight.getCurrentPosition());

        }
        telemetry.update();
    }
}

