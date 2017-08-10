package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

public class SwerveDriveHardware {

    boolean isCarMode = false;
    boolean isForward = true;
    boolean isTurn = false;
    boolean isTrueCar = false;

    //Booleans for Debugging
    boolean isTestingFL = false;
    boolean isTestingFR = false;
    boolean isTestingBL = false;
    boolean isTestingBR = false;

    public boolean use_imu = true;
    public boolean use_encoder = true;
    public boolean fast_mode = false;
    public boolean straight_mode = false;
    public double target_heading = 0.0;
    public float leftPower = 0;
    public float rightPower = 0;
    public int leftCnt = 0; // left motor target counter
    public int rightCnt = 0; // right motor target counter

    final static int ONE_ROTATION = 1120; // for AndyMark-40 motor encoder one rotation
    final static double RROBOT = 6.63;  // number of wheel turns to get chassis 360-degree turn
    final static double INCHES_PER_ROTATION = 12.57; // inches per chassis motor rotation based on 16/24 gear ratio
    final static double GYRO_ROTATION_RATIO_L = 0.65; // 0.83; // Ratio of Gyro Sensor Left turn to prevent overshooting the turn.
    final static double GYRO_ROTATION_RATIO_R = 0.65; // 0.84; // Ratio of Gyro Sensor Right turn to prevent overshooting the turn.
    final static double NAVX_ROTATION_RATIO_L = 0.65; // 0.84; // Ratio of NavX Sensor Left turn to prevent overshooting the turn.
    final static double NAVX_ROTATION_RATIO_R = 0.65; // 0.84; // Ratio of NavX Sensor Right turn to prevent overshooting the turn.
    final static double DRIVE_RATIO_L = 1.0; //control veering by lowering left motor power
    final static double DRIVE_RATIO_R = 1.0; //control veering by lowering right motor power

    double motorPowerLeft;
    double motorPowerRight;
    double motorPowerTurn;
    double servoPosFL;
    double servoPosFR;
    double servoPosBL;
    double servoPosBR;

    /* Public OpMode members. */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorEncoder = null;

    public Servo servoFrontLeft = null;
    public Servo servoFrontRight = null;
    public Servo servoBackLeft = null;
    public Servo servoBackRight = null;

    final static double SERVO_FL_FORWARD_POSITION = 0.6;
    final static double SERVO_FR_FORWARD_POSITION = 0.41;
    final static double SERVO_BL_FORWARD_POSITION = 0.40;
    final static double SERVO_BR_FORWARD_POSITION = 0.63;

    final static double SERVO_FL_STRAFE_POSITION = 0.09;
    final static double SERVO_FR_STRAFE_POSITION = 0.96;
    final static double SERVO_BL_STRAFE_POSITION = 0.92;
    final static double SERVO_BR_STRAFE_POSITION = 0.12;

    final static double SERVO_FL_TURN_POSITION = 0.40;
    final static double SERVO_FR_TURN_POSITION = 0.64;
    final static double SERVO_BL_TURN_POSITION = 0.63;
    final static double SERVO_BR_TURN_POSITION = 0.43;


    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public SwerveDriveHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Define and Initialize Motors
        motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
        motorBackRight = hwMap.dcMotor.get("motorBackRight");
        motorEncoder = hwMap.dcMotor.get("motorEncoder");

        servoFrontRight = hwMap.servo.get("servoFrontRight");
        servoFrontLeft = hwMap.servo.get("servoFrontLeft");
        servoBackLeft = hwMap.servo.get("servoBackLeft");
        servoBackRight = hwMap.servo.get("servoBackRight");

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power and set all servos to central position
        // May want to change servo #'s to the value where all wheels are pointing forward.
        servoFrontLeft.setPosition(SERVO_FL_FORWARD_POSITION);
        servoFrontRight.setPosition(SERVO_FR_FORWARD_POSITION);
        servoBackLeft.setPosition(SERVO_BL_FORWARD_POSITION);
        servoBackRight.setPosition(SERVO_BR_FORWARD_POSITION);

        servoPosFL = SERVO_FL_FORWARD_POSITION;
        servoPosFR = SERVO_FR_FORWARD_POSITION;
        servoPosBL = SERVO_BL_FORWARD_POSITION;
        servoPosBR = SERVO_BR_FORWARD_POSITION;

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorEncoder.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public double imu_heading() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void driveTT(double lp, double rp) {
        if(!fast_mode && straight_mode) { // expect to go straight
            if (use_imu) {
                double cur_heading = imu_heading();
                if (cur_heading - target_heading > 0.7) { // crook to left,  slow down right motor
                    if (rp > 0) rp *= 0.5;
                    else lp *= 0.5;
                } else if (cur_heading - target_heading < -0.7) { // crook to right, slow down left motor
                    if (lp > 0) lp *= 0.5;
                    else rp *= 0.5;
                }
            }
        }
        if (Math.abs(rp) > 0.3 && Math.abs(lp) > 0.3) {
            motorFrontRight.setPower(rp * DRIVE_RATIO_R);
            motorBackRight.setPower(rp * DRIVE_RATIO_R);
            motorFrontLeft.setPower(lp * DRIVE_RATIO_L);
            motorBackLeft.setPower(lp * DRIVE_RATIO_L);
        }
        else{
            motorFrontRight.setPower(rp);
            motorBackRight.setPower(rp);
            motorFrontLeft.setPower(lp);
            motorBackLeft.setPower(lp);
        }
        if (use_encoder) {
            motorEncoder.setPower(Math.max(lp, rp));
        }
    }

    void stop_chassis() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorEncoder.setPower(0);
    }
    void reset_chassis()  {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftCnt = 0;
        rightCnt = 0;
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void stop_tobot() {
        stop_chassis();

    }

    public void run_until_encoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {
        //motorFR.setTargetPosition(rightCnt);
        //motorBL.setTargetPosition(leftCnt);
        //motorBL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //motorFR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //waitOneFullHardwareCycle();
        ElapsedTime     runtime = new ElapsedTime();

        int leftTC1 = leftCnt;
        int rightTC1 = rightCnt;
        int leftTC2 = 0;
        int rightTC2 = 0;
        int leftTC0 = 0;
        int rightTC0 = 0;
        double initLeftPower = leftPower;
        double initRightPower = rightPower;
        if (leftPower > 0.4 && leftTC1 > 600 && !fast_mode) {
            leftTC2 = 450;
            leftTC0 = 50;
            leftTC1 -= 500;
        }
        if (rightPower > 0.4 && rightTC1 > 600 && !fast_mode) {
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
        //while (motorFR.isBusy() || motorBL.isBusy()) {
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
        DcMotor mt = motorEncoder;
        if (leftPower < 0) {
            //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
            return (mt.getCurrentPosition() <= p_count);
        } else {
            //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
            return (mt.getCurrentPosition() >= p_count);
        }
    } // has_left_drive_encoder_reached

    boolean has_right_drive_encoder_reached(double p_count) {
        DcMotor mt = motorEncoder;
        if (rightPower < 0) {
            return (mt.getCurrentPosition() <= p_count);
        } else {
            return (mt.getCurrentPosition() >= p_count);
        }

    } // has_right_drive_encoder_reached

    boolean have_drive_encoders_reached(double p_left_count, double p_right_count) {
        DcMotor mt = motorEncoder;
        boolean l_return = false;
        if (has_left_drive_encoder_reached(p_left_count) && has_right_drive_encoder_reached(p_right_count)) {
            l_return = true;
        } else if (has_left_drive_encoder_reached(p_left_count)) { // shift target encoder value from right to left
            double diff = Math.abs(p_right_count - mt.getCurrentPosition()) / 2;
            if (leftPower < 0) {
                leftCnt -= diff;
            } else {
                leftCnt += diff;
            }
            if (rightPower < 0) {
                rightCnt += diff;
            } else {
                rightCnt -= diff;
            }
        } else if (has_right_drive_encoder_reached(p_right_count)) { // shift target encoder value from left to right
            double diff = Math.abs(p_left_count - mt.getCurrentPosition()) / 2;
            if (rightPower < 0) {
                rightCnt -= diff;
            } else {
                rightCnt += diff;
            }
            if (leftPower < 0) {
                leftCnt += diff;
            } else {
                leftCnt -= diff;
            }
        }
        return l_return;
    } // have_encoders_reached

    public void TurnRightD(double power, double degree) throws InterruptedException {

        double adjust_degree_gyro = GYRO_ROTATION_RATIO_R * (double) degree;
        double adjust_degree_navx = NAVX_ROTATION_RATIO_R * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        ElapsedTime     runtime = new ElapsedTime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorFrontLeft.getCurrentPosition();
        int rightEncode = motorFrontRight.getCurrentPosition();
        leftCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);
        rightCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);

        leftPower = (float) -power;
        rightPower = (float) power;

        servoFrontLeft.setPosition(SERVO_FL_TURN_POSITION);
        servoFrontRight.setPosition(SERVO_FR_TURN_POSITION);
        servoBackLeft.setPosition(SERVO_BL_TURN_POSITION);
        servoBackRight.setPosition(SERVO_BR_TURN_POSITION);

        sleep(200);

        leftCnt += leftEncode;
        rightCnt += rightEncode;

        DbgLog.msg(String.format("imu Right Turn %.2f degree with %.2f power.", degree, power));
        if (use_imu) {
            current_pos = imu_heading();
            target_heading = current_pos - adjust_degree_navx;
            if (target_heading <= -180) {
                target_heading += 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos <= 0)) {
                current_pos += 360;
            }
            while ((current_pos >= target_heading) && (runtime.seconds() < 4.0)) {
                current_pos = imu_heading();
                // DbgLog.msg(String.format("imu current/target heading = %.2f/%.2f",current_pos,target_heading));

                if (heading_cross_zero && (current_pos <= 0)) {
                    current_pos += 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else {
            if (use_encoder) {
                run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(leftPower, rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
        sleep(300);
        if(isForward){
            servoFrontLeft.setPosition(SERVO_FL_FORWARD_POSITION);
            servoFrontRight.setPosition(SERVO_FR_FORWARD_POSITION);
            servoBackLeft.setPosition(SERVO_BL_FORWARD_POSITION);
            servoBackRight.setPosition(SERVO_BR_FORWARD_POSITION);
        }
        else{
            servoFrontLeft.setPosition(SERVO_FL_STRAFE_POSITION);
            servoFrontRight.setPosition(SERVO_FR_STRAFE_POSITION);
            servoBackLeft.setPosition(SERVO_BL_STRAFE_POSITION);
            servoBackRight.setPosition(SERVO_BR_STRAFE_POSITION);
        }
        if (!fast_mode)
            sleep(135);
    }
    public void TurnLeftD(double power, double degree) throws InterruptedException {
        double adjust_degree_gyro = GYRO_ROTATION_RATIO_L * (double) degree;
        double adjust_degree_navx = NAVX_ROTATION_RATIO_L * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        ElapsedTime     runtime = new ElapsedTime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS)
        int leftEncode = motorFrontLeft.getCurrentPosition();
        int rightEncode = motorFrontRight.getCurrentPosition();
        leftCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);
        rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);

        leftPower = (float) power;
        rightPower = (float) -power;

        servoFrontLeft.setPosition(SERVO_FL_TURN_POSITION);
        servoFrontRight.setPosition(SERVO_FR_TURN_POSITION);
        servoBackLeft.setPosition(SERVO_BL_TURN_POSITION);
        servoBackRight.setPosition(SERVO_BR_TURN_POSITION);

        sleep(200);

        leftCnt += leftEncode;
        rightCnt += rightEncode;


        DbgLog.msg(String.format("imu Left Turn %.2f degree with %.2f power.", degree, power));
        if (use_imu) {
            current_pos = imu_heading();
            target_heading = current_pos + adjust_degree_navx;
            if (target_heading >= 180) {
                target_heading -= 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos >= 0)) {
                current_pos -= 360;
            }
            DbgLog.msg(String.format("imu Left Turn curr/target pos = %.2f/%.2f.", current_pos, target_heading));
            while ((current_pos <= target_heading) && (runtime.seconds() < 5.0)) {
                current_pos = imu_heading();
                if (heading_cross_zero && (current_pos >= 0)) {
                    current_pos -= 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else {
            if (use_encoder) {
                run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(leftPower, rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
        sleep(300);
        if(isForward){
            servoFrontLeft.setPosition(SERVO_FL_FORWARD_POSITION);
            servoFrontRight.setPosition(SERVO_FR_FORWARD_POSITION);
            servoBackLeft.setPosition(SERVO_BL_FORWARD_POSITION);
            servoBackRight.setPosition(SERVO_BR_FORWARD_POSITION);
        }
        else{
            servoFrontLeft.setPosition(SERVO_FL_STRAFE_POSITION);
            servoFrontRight.setPosition(SERVO_FR_STRAFE_POSITION);
            servoBackLeft.setPosition(SERVO_BL_STRAFE_POSITION);
            servoBackRight.setPosition(SERVO_BR_STRAFE_POSITION);
        }
        if (!fast_mode)
            sleep(135);
    }

    public void StraightR(double power, double n_rotations) throws InterruptedException {
        DcMotor mt = motorEncoder;
        straight_mode = true;
        reset_chassis();
        // set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = mt.getCurrentPosition();
        int rightEncode = mt.getCurrentPosition();
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftCnt = (int) (ONE_ROTATION * n_rotations);
        rightCnt = (int) (ONE_ROTATION * n_rotations);
        leftPower = rightPower = (float) power;
        if (power < 0) { // move backward
            leftCnt = leftEncode - leftCnt;
            rightCnt = rightEncode - rightCnt;
        } else {
            leftCnt += leftEncode;
            rightCnt += rightEncode;
        }
        run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
        straight_mode = false;
        if (!fast_mode)
            sleep(135);
    }

    public void StraightIn(double power, double in) throws InterruptedException {
        if (use_imu) {
            target_heading = imu_heading();
        }
        if (use_encoder) {
            double numberR = in / INCHES_PER_ROTATION;
            StraightR(power, numberR);
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
}
