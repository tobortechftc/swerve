package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    double motorPowerLeft;
    double motorPowerRight;
    double servoPosFL;
    double servoPosFR;
    double servoPosBL;
    double servoPosBR;

    /* Public OpMode members. */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    public Servo servoFrontLeft = null;
    public Servo servoFrontRight = null;
    public Servo servoBackLeft = null;
    public Servo servoBackRight = null;

    final static double SERVO_FL_FORWARD_POSITION = 0.55;
    final static double SERVO_FR_FORWARD_POSITION = 0.50;
    final static double SERVO_BL_FORWARD_POSITION = 0.47;
    final static double SERVO_BR_FORWARD_POSITION = 0.55;

    final static double SERVO_FL_STRAFE_POSITION = 0.05;
    final static double SERVO_FR_STRAFE_POSITION = 0.95;
    final static double SERVO_BL_STRAFE_POSITION = 0.95;
    final static double SERVO_BR_STRAFE_POSITION = 0.05;

    final static double SERVO_FL_TURN_POSITION = 0.5;
    final static double SERVO_FR_TURN_POSITION = 0.5;
    final static double SERVO_BL_TURN_POSITION = 0.5;
    final static double SERVO_BR_TURN_POSITION = 0.5;

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

        // Define and Initialize Motors
        motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
        motorBackRight = hwMap.dcMotor.get("motorBackRight");

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

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
}
