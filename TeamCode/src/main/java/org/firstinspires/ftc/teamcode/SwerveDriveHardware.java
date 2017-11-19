package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.security.InvalidParameterException;

import static java.lang.Thread.sleep;

public class SwerveDriveHardware {

    // define all switches to turn on/off hardware each component
    public boolean use_swerve = true;   // use four motors and four servos for chassis
    public boolean use_minibot = false; // use motorFrontLeft and motorFrontRight only for chassis
    public boolean use_Vuforia = true;
    public boolean use_camera = false;
    public boolean use_imu = true;
    public boolean use_encoder = true;
    public boolean use_color_sensor = false;
    public boolean use_range_sensor = false;
    public boolean use_relic_grabber = false;
    public boolean use_glyph_grabber = false;
    public boolean use_arm = false;
    public boolean use_test_servo = false;
    public boolean use_test_motor = false;

    public boolean fast_mode = true;
    public boolean straight_mode = false;

    boolean isTesting = true;

    boolean enoughToSnake = true; //See if turning radius doesn't extend to inside the robot
    boolean isSnakingLeft = false; //See if the snake drive is turning to the left

    //Booleans for Debugging
    boolean isTestingFL = false;
    boolean isTestingFR = false;
    boolean isTestingBL = false;
    boolean isTestingBR = false;

    boolean isRedBall = false;
    boolean isBlueBall = false;
    boolean gg_slider_encoder_ok = false;
    boolean gg_rotator_encoder_ok = false;
    boolean gg_top_close = false;
    boolean gg_bottom_close = false;
    boolean is_gg_upside_down = false;


    public double target_heading = 0.0;
    public double mt_glyph_slider_pw = 0.0;
    public float leftPower = 0;
    public float rightPower = 0;
    public int leftCnt = 0; // left motor target counter
    public int rightCnt = 0; // right motor target counter

    final static int ONE_ROTATION_60 = 1680; // AndyMark NeveRest-60
    final static int ONE_ROTATION_40 = 1120; // AndyMark NeveRest-40
    final static int ONE_ROTATION_20 = 560; // AndyMark NeveRest-20
    final static int ONE_ROTATION = 538; // for new AndyMark-20 motor encoder one rotation
    final static double RROBOT = 6.63;  // number of wheel turns to get chassis 360-degree turn
    final static double INCHES_PER_ROTATION = 12.69; // inches per chassis motor rotation based on 1:1 gear ratio

    final static double IMU_ROTATION_RATIO_L = 0.6; // 0.84; // Ratio of IMU Sensor Left turn to prevent overshooting the turn.
    final static double IMU_ROTATION_RATIO_R = 0.6; // 0.84; // Ratio of IMU Sensor Right turn to prevent overshooting the turn.

    final static double INIT_DRIVE_RATIO_FL = 0.998; //control veering by lowering left motor power
    final static double INIT_DRIVE_RATIO_FR = 0.978; //control veering by lowering right motor power
    final static double INIT_DRIVE_RATIO_BL = 1.0; //control veering by lowering left motor power
    final static double INIT_DRIVE_RATIO_BR = 0.982; //control veering by lowering right motor power


    final static double WIDTH_BETWEEN_WHEELS = 12;
    final static double LENGTH_BETWEEN_WHEELS = 12;
    final static double MIN_TURNING_RADIUS = 13;
    final static double MAX_TURNING_RADIUS = 100;

    final static double GG_SLIDE_INCHES_PER_ROTATION = 6.5; // glyph slider moves # inches per motor rotation
    final static double GG_SLIDE_MAX_COUNT = 4150; // ~14 inches
    final static int GG_SLIDE_INIT = 50;


    final static int RED_BALL_MIN = -94;
    final static int RED_BALL_MAX = -36;
    final static int BLUE_BALL_MIN = 12;
    final static int BLUE_BALL_MAX = 66;

    final static double SV_SHOULDER_INIT = 0.4978;
    final static double SV_SHOULDER_DOWN = 0.4889;
    final static double SV_SHOULDER_LEFT = 0.6294;
    final static double SV_SHOULDER_RIGHT = 0.15;

    final static double SV_ELBOW_UP = 0.95;
    final static double SV_ELBOW_DOWN = 0.48;

    final static double SV_GLYPH_GRABBER_TOP_INIT = 0.4;
    final static double SV_GLYPH_GRABBER_TOP_OPEN = 0.5;
    final static double SV_GLYPH_GRABBER_TOP_CLOSED = 0.6;

    final static double SV_GLYPH_GRABBER_BOTTOM_INIT = 0.65;
    final static double SV_GLYPH_GRABBER_BOTTOM_OPEN = 0.615;
    final static double SV_GLYPH_GRABBER_BOTTOM_CLOSED = 0.46;
    final static double SV_RELIC_GRABBER_INIT = 0.76;
    final static double SV_RELIC_ARM_INIT = 0.5;
    final static double SV_RELIC_ARM_UP = 0.5;
    final static double SV_RELIC_ARM_DOWN = 0.5;
    final static double GG_SLIDE_UP_POWER = 0.8;
    final static double GG_SLIDE_DOWN_POWER = -0.5;


    double motorPowerLeft;
    double motorPowerRight;
    double motorPowerTurn;
    double drivePowerRatio = 0.8;

    double servoPosFL;
    double servoPosFR;
    double servoPosBL;
    double servoPosBR;

    double leftServoAngle;
    double rightServoAngle;
    double r_Value;
    double thetaOneCalc;
    double thetaTwoCalc;
    double insideWheelsMod;
    double outsideWheelsMod;

    int targetColumn;

    double blue = 0;
    double red = 0;

    int orig_rot_pos = 0;
    int target_rot_pos = 0;
    int init_gg_slider_pos = GG_SLIDE_INIT;
    int target_gg_slider_pos = 0;
    int gg_layer = 0;
    int max_gg_layer = 2;
    int [] layer_positions = {GG_SLIDE_INIT, ONE_ROTATION_60+GG_SLIDE_INIT, 2*ONE_ROTATION_60+GG_SLIDE_INIT};

    double DRIVE_RATIO_FL = INIT_DRIVE_RATIO_FL; //control veering by lowering left motor power
    double DRIVE_RATIO_FR = INIT_DRIVE_RATIO_FR;//control veering by lowering right motor power
    double DRIVE_RATIO_BL = INIT_DRIVE_RATIO_BL; //control veering by lowering left motor power
    double DRIVE_RATIO_BR = INIT_DRIVE_RATIO_BR;//control veering by lowering right motor power



    /* Public OpMode members. */
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    public DcMotor mt_relic_slider = null;
    public DcMotor mt_test = null;

    public DcMotor mt_glyph_rotator = null;
    public DcMotor mt_glyph_slider = null;
    public Servo servoFrontLeft = null;
    public Servo servoFrontRight = null;
    public Servo servoBackLeft = null;
    public Servo servoBackRight = null;

    public Servo sv_shoulder;
    public Servo sv_elbow;

    public Servo sv_glyph_grabber_top = null;
    public Servo sv_glyph_grabber_bottom = null;

    public Servo sv_relic_grabber = null;
    public Servo sv_relic_arm = null;
    public Servo sv_test = null;

    public ColorSensor colorSensor = null;
    public ModernRoboticsI2cRangeSensor rangeSensorBack = null;
    public ModernRoboticsI2cRangeSensor rangeSensorLeft = null;
    public SwerveUtilLOP.Camera camera = null;

    ElapsedTime runtime = new ElapsedTime();

    final static double SERVO_FL_FORWARD_POSITION = 0.48;
    final static double SERVO_FR_FORWARD_POSITION = 0.45;
    final static double SERVO_BL_FORWARD_POSITION = 0.48;
    final static double SERVO_BR_FORWARD_POSITION = 0.5;

    final static double SERVO_FL_STRAFE_POSITION = SERVO_FL_FORWARD_POSITION + 0.475;
    final static double SERVO_FR_STRAFE_POSITION = SERVO_FR_FORWARD_POSITION - 0.475;
    final static double SERVO_BL_STRAFE_POSITION = SERVO_BL_FORWARD_POSITION - 0.475;
    final static double SERVO_BR_STRAFE_POSITION = SERVO_BR_FORWARD_POSITION + 0.475;

    final static double SERVO_FL_TURN_POSITION = SERVO_FL_FORWARD_POSITION - (0.475/2);
    final static double SERVO_FR_TURN_POSITION = SERVO_FR_FORWARD_POSITION + (0.475/2);
    final static double SERVO_BL_TURN_POSITION = SERVO_BL_FORWARD_POSITION + (0.475/2);
    final static double SERVO_BR_TURN_POSITION = SERVO_BR_FORWARD_POSITION - (0.475/2);

    enum CarMode {
        CAR,
        STRAIGHT,
        CRAB,
        TURN
    }
    CarMode cur_mode = CarMode.CAR;
    CarMode old_mode = CarMode.STRAIGHT;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    VuforiaLocalizer vuforia;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaLocalizer.Parameters parameters;

    /* Constructor */
    public SwerveDriveHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        if (use_Vuforia) {
            int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = "AaaZDWL/////AAAAGYIaD+Gn/UUDhEiR/gcOJxdEJlKEpCOKSLPfhYJfYthUNZ0vnEGm0VGPutkNgRq8bq1ufm3eAySnLhkJQ7d4w6VDT7os5FGPEOGPfsIWMYNAFMdX+wlJo2JCyljeSxQtXUd/YileyfYKBXOl2uFA4KnStCC9WkYTUBrAof3H7RGKorzYixDeOpbmCsf25rayjtAUQrKCwG4j6P5rRdxy7SC//v4VC6NirNwgJ/xn1r02/jbx8vUDrDODGyut9iLk06IzMnrq/P01yKOp48clTw0WIKNmVT7WUQweXy+E1w6xwFplTlPkjC+gzerDOpxHPqYg8RusWD2Y/IMlmnk1yzJba1B9Xf9Ih6BJbm/fVwL4";

            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        }
        if (use_camera) {
            if (!use_Vuforia) {
                throw new IllegalStateException("use_camera cannot be flagged as true without use_Vuforia also being true!");
            }
            this.camera = new SwerveUtilLOP.Camera(this.vuforia);
        }

        if (use_imu) {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
            imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            imuParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            imuParameters.loggingEnabled = true;
            imuParameters.loggingTag = "IMU";
            imuParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".

            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(imuParameters);
        }

        if (use_color_sensor) {
            colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
            colorSensor.enableLed(true);
        }
        if (use_range_sensor) {
            rangeSensorBack = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorBack");
            rangeSensorLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorLeft");
        }
        if (use_test_servo) {
            // sv_test = hwMap.servo.get("sv_test");
        }
        if (use_test_motor) {
            mt_test = hwMap.dcMotor.get("mt_test");
            mt_test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mt_test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            orig_rot_pos = mt_test.getCurrentPosition();
        }
        if (use_relic_grabber) {
            sv_relic_arm = hwMap.servo.get("sv_relic_arm");
            sv_relic_arm.setPosition(SV_RELIC_ARM_INIT);
            sv_relic_grabber = hwMap.servo.get("sv_relic_grabber");
            sv_relic_grabber.setPosition(SV_RELIC_GRABBER_INIT);
            // mt_relic_slider = hwMap.dcMotor.get("mt_relic_slider");
        }
        if (use_glyph_grabber) {
            sv_glyph_grabber_bottom = hwMap.servo.get("sv_grabber_bottom");
            sv_glyph_grabber_bottom.setPosition(SV_GLYPH_GRABBER_BOTTOM_INIT);

            sv_glyph_grabber_top = hwMap.servo.get("sv_grabber_top");
            sv_glyph_grabber_top.setPosition(SV_GLYPH_GRABBER_TOP_INIT);

            mt_glyph_rotator = hwMap.dcMotor.get("mt_glyph_rotator");
            mt_glyph_rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mt_glyph_rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mt_glyph_rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mt_glyph_slider = hwMap.dcMotor.get("mt_glyph_slider");
            mt_glyph_slider.setDirection(DcMotor.Direction.REVERSE);
            mt_glyph_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_glyph_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mt_glyph_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (use_minibot) {
            motorFrontLeft = hwMap.dcMotor.get("left_drive");
            motorFrontRight = hwMap.dcMotor.get("right_drive");

            motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
            motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            use_swerve = false;
        }
        else if (use_swerve) {
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

            // Set all motors to run with encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (use_arm) {
            sv_elbow = hwMap.servo.get("sv_elbow");
            sv_shoulder = hwMap.servo.get("sv_shoulder");
            sv_elbow.setPosition(SV_ELBOW_UP);
            sv_shoulder.setPosition(SV_SHOULDER_INIT);
        }

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
