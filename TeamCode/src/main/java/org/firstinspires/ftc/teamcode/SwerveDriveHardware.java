package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SwerveDriveHardware {

    boolean isForward = true;

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

    public DcMotor servoFrontLeft = null;
    public DcMotor servoFrontRight = null;
    public DcMotor servoBackLeft = null;
    public DcMotor servoBackRight = null;

    final static double SERVO_FL_FORWARD_POSITION = 0.05;
    final static double SERVO_FR_FORWARD_POSITION = 0.05;
    final static double SERVO_BL_FORWARD_POSITION = 0.05;
    final static double SERVO_BR_FORWARD_POSITION = 0.05;

    final static double SERVO_FL_STRAFE_POSITION = 0.5;
    final static double SERVO_FR_STRAFE_POSITION = 0.5;
    final static double SERVO_BL_STRAFE_POSITION = 0.5;
    final static double SERVO_BR_STRAFE_POSITION = 0.5;

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
        motorBackLeft = hwMap.dcMotor.get("motorBackRight");

        servoFrontRight = hwMap.dcMotor.get("servoFrontRight");
        servoFrontLeft = hwMap.dcMotor.get("servoFrontLeft");
        servoBackLeft = hwMap.dcMotor.get("servoBackLeft");
        servoBackLeft = hwMap.dcMotor.get("servoBackRight");

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power and set all servos to central position
        // May want to change servo #'s to the value where all wheels are pointing forward.
        servoFrontLeft.setPower(SERVO_FL_FORWARD_POSITION);
        servoFrontRight.setPower(SERVO_FR_FORWARD_POSITION);
        servoBackLeft.setPower(SERVO_BL_FORWARD_POSITION);
        servoBackRight.setPower(SERVO_BR_FORWARD_POSITION);

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
