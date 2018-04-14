package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Thread.sleep;

public class SwerveDriveHardware {

    // define all switches to turn on/off hardware each component
    public boolean use_verbose = false;
    public boolean use_swerve = true;   // use four motors and four servos for chassis
    public boolean use_newbot = false;   // use four motors and four servos for new chassis
    public boolean use_newbot_v2 = false;
    public boolean use_relic_elbow = false;
    public boolean use_front_drive_only = false;
    public boolean use_intake = false;
    public boolean use_dumper = false;
    public boolean use_dumper_gate = true;
    public boolean use_minibot = false; // use motorFrontLeft and motorFrontRight only for chassis
    public boolean use_Vuforia = true;
    public boolean use_camera = false;
    public boolean use_imu = true;
    public boolean use_imu2 = false;
    public boolean use_encoder = true;
    public boolean use_color_sensor = false;
    public boolean use_range_sensor = false;
    public boolean use_relic_grabber = true;
    public boolean use_relic_slider = true;
    public boolean use_glyph_grabber = false;
    public boolean use_arm = false;
    public boolean use_test_servo = false;
    public boolean use_test_motor = false;
    public boolean use_proximity_sensor = false;
    public boolean use_front_arm = false;
    public boolean servo_tune_up = false;

    public SwerveUtilLOP.TeamColor allianceColor = SwerveUtilLOP.TeamColor.BLUE; // default blue team

    public boolean fast_mode = true; //Controls how "rushed" autonomous actions are
    public boolean straight_mode = false;
    public boolean deliver_mode = false; //Affects gamepad1's controls, switches the function of the sticks

    boolean isTesting = false;
    boolean needsUpdate = false;
    boolean tried_clockwise = false; //For determining how to correct the glyph during autonomous collection
    boolean snaked_left = false;

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
    boolean stop_on_dump = false;
    boolean bump_detected = false;


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

    final static double IMU_ROTATION_RATIO_L = 0.8; // 0.84; // Ratio of IMU Sensor Left turn to prevent overshooting the turn.
    final static double IMU_ROTATION_RATIO_R = 0.8; // 0.84; // Ratio of IMU Sensor Right turn to prevent overshooting the turn.

    final static double INIT_DRIVE_RATIO_FL = 0.998; //control veering by lowering left motor power
    final static double INIT_DRIVE_RATIO_FR = 0.978; //control veering by lowering right motor power
    final static double INIT_DRIVE_RATIO_BL = 1.0; //control veering by lowering left motor power
    final static double INIT_DRIVE_RATIO_BR = 0.982; //control veering by lowering right motor power


    final static double WIDTH_BETWEEN_WHEELS = 12;
    final static double LENGTH_BETWEEN_WHEELS = 12;
    final static double NB_WIDTH_BETWEEN_WHEELS = 12;
    final static double NB_LENGTH_BETWEEN_WHEELS = 11;
    final static double DISTANCE_TO_CENTER_OF_GLYPH = 5.9;
    final static double NB_DISTANCE_FOR_ORBIT = 6.5;
    final static double MIN_TURNING_RADIUS = 13;
    final static double MAX_TURNING_RADIUS = 100;
    final static double THETA_FRONT = (Math.atan(DISTANCE_TO_CENTER_OF_GLYPH / (0.5 * WIDTH_BETWEEN_WHEELS))) * (180/Math.PI);
    final static double THETA_BACK = (Math.atan((DISTANCE_TO_CENTER_OF_GLYPH + LENGTH_BETWEEN_WHEELS) / (0.5 * WIDTH_BETWEEN_WHEELS))) * (180/Math.PI);
    final static double NB_THETA_FRONT = (Math.atan(NB_DISTANCE_FOR_ORBIT / (0.5 * NB_WIDTH_BETWEEN_WHEELS))) * (180/Math.PI);
    final static double NB_THETA_BACK = (Math.atan((NB_DISTANCE_FOR_ORBIT + NB_LENGTH_BETWEEN_WHEELS) / (0.5 * NB_WIDTH_BETWEEN_WHEELS))) * (180/Math.PI);
    static double SPOT_TURN_ANGLE_OFFSET = (Math.atan((0.5*LENGTH_BETWEEN_WHEELS)/(0.5*WIDTH_BETWEEN_WHEELS))) * (1/(Math.PI));
    static double NB_SPOT_TURN_ANGLE_OFFSET = (Math.atan((0.5*NB_LENGTH_BETWEEN_WHEELS)/(0.5*NB_WIDTH_BETWEEN_WHEELS))) * (1/(Math.PI));
    final static long SERVO_TURN_TIME = 200;

    final static double GG_SLIDE_INCHES_PER_ROTATION = 6.5; // glyph slider moves # inches per motor rotation
    final static double GG_SLIDE_MAX_COUNT = 4700; // ~14 inches
    final static int LIFT_INIT_COUNT = 30;
    final static int LIFT_MAX_COUNT = 2470+LIFT_INIT_COUNT;
    final static int GG_SLIDE_INIT = 10;
    final static int RELIC_SLIDE_MAX = -8800;



    final static int RED_BALL_MIN = -600;
    final static int RED_BALL_MAX = -15;
    final static int BLUE_BALL_MIN = 12;
    final static int BLUE_BALL_MAX = 600;

    final static double SV_SHOULDER_INIT = 0.47;
    final static double SV_SHOULDER_DOWN = 0.46;
    final static double SV_SHOULDER_LEFT_3 = 0.6294;
    final static double SV_SHOULDER_LEFT_2 = 0.585;
    final static double SV_SHOULDER_LEFT_1 = 0.535;
    final static double SV_SHOULDER_RIGHT_3 = 0.30;
    final static double SV_SHOULDER_RIGHT_2 = 0.40;
    final static double SV_SHOULDER_RIGHT_1 = 0.45;

    final static double SV_ELBOW_UP = 0.997;
    final static double SV_ELBOW_DOWN = 0.44;
    final static double SV_ELBOW_DOWN_HIT = 0.43;

    final static double SV_RIGHT_ARM_UP = 0.11;
    final static double SV_RIGHT_ARM_DOWN = 0.73;
    final static double SV_RIGHT_ARM_UP_NB = 0.18;
    final static double SV_RIGHT_ARM_DOWN_NB = 0.6628;
    final static double SV_LEFT_ARM_UP_NB = 0.943;
    final static double SV_LEFT_ARM_DOWN_NB = 0.359;
    final static double SV_FRONT_ARM_IN = 0.83;
    final static double SV_FRONT_ARM_OUT = 0.43;
    final static double SV_JKICKER_UP = 0.47;
    final static double SV_JKICKER_RIGHT1 = 0.57;
    final static double SV_JKICKER_RIGHT2 = 0.65;
    final static double SV_JKICKER_RIGHT = 0.76;
    final static double SV_JKICKER_LEFT1 = 0.37;
    final static double SV_JKICKER_LEFT2 = 0.27;
    final static double SV_JKICKER_LEFT = 0.17;
    final static double SV_JKICKER_INIT = 0.82;


    final static double SV_GLYPH_GRABBER_TOP_INIT = 0.3; //from .275
    final static double SV_GLYPH_GRABBER_TOP_OPEN = 0.38;
    final static double SV_GLYPH_GRABBER_TOP_HALF_CLOSED = 0.445;
    final static double SV_GLYPH_GRABBER_TOP_CLOSED = 0.65; //from .57
    final static double SV_GLYPH_GRABBER_BOTTOM_INIT = 0.65;
    final static double SV_GLYPH_GRABBER_BOTTOM_OPEN = 0.528;
    final static double SV_GLYPH_GRABBER_BOTTOM_HALF_CLOSED = 0.485;
    final static double SV_GLYPH_GRABBER_BOTTOM_CLOSED = 0.358; // 0.405 0.037

    final static double SV_RELIC_GRABBER_INIT = 0.5039;
    final static double SV_RELIC_GRABBER_CLOSE = 0.5006;
    final static double SV_RELIC_GRABBER_OPEN = 0.1822;
    final static double SV_RELIC_GRABBER_INIT_NB = 0.86;
    final static double SV_RELIC_GRABBER_CLOSE_NB = 0.8;
    final static double SV_RELIC_GRABBER_OPEN_NB = 0.25;
    final static double SV_RELIC_GRABBER_OPEN_W_NB = 0.15;
    final static double SV_RELIC_ARM_INIT = 0.1;
    final static double SV_RELIC_ARM_UP = 0.7;
    final static double SV_RELIC_ARM_MIDDLE = 0.55;
    final static double SV_RELIC_ARM_DOWN = 0.2;
    final static double SV_RELIC_ARM_DOWN_R = 0.27; // down and ready for release
    final static double SV_RELIC_WRIST_INIT = 0.329;
    final static double SV_RELIC_WRIST_UP = 0.99;
    final static double SV_RELIC_WRIST_MIDDLE = 0.78;
    final static double SV_RELIC_WRIST_DOWN = 0.379;
    final static double SV_RELIC_WRIST_DOWN_R = 0.4; // down and ready for release
    final static double SV_RELIC_WRIST_DOWN_AUTO = SV_RELIC_WRIST_DOWN;

    final static double SV_RELIC_ELBOW_INIT = 0.6167;
    final static double SV_RELIC_ELBOW_UP = 0.5689;
    final static double SV_RELIC_ELBOW_FLAT = 0.5117;
    final static double SV_RELIC_ELBOW_DOWN = 0.5;
    final static double SV_DUMPER_INIT = 0.692;
    final static double SV_DUMPER_DOWN = 0.692;
    final static double SV_DUMPER_LIFT = 0.599;
    final static double SV_DUMPER_HALF_UP = 0.551;
    final static double SV_DUMPER_UP = 0.204;
    final static double SV_DUMPER_DUMP = 0.214;
    final static double SV_INTAKE_GATE_INIT = 0.844;
    final static double SV_INTAKE_GATE_UP = 0.61;
    final static double SV_INTAKE_GATE_MID = 0.5;
    final static double SV_INTAKE_GATE_DOWN = 0.217;
    final static double SV_DUMPER_GATE_INIT = 0.25;
    final static double SV_DUMPER_GATE_UP = 0.25;
    final static double SV_DUMPER_GATE_DOWN = 0.62;

    final static double GG_SLIDE_UP_POWER = 1.0;
    final static double GG_SLIDE_DOWN_POWER = -0.9;


    double motorPowerLeft;
    double motorPowerRight;
    double motorPowerTurn;
    double drivePowerRatio = 0.5; //Controls the upper cap on drive speed
    double intakeRatio = 1.0;
    float drivePower = 0; //Controls the throttling of the drive

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

    int targetColumn = -1;

    double blue = 0;
    double red = 0;

    int orig_rot_pos = 0;
    int target_rot_pos = 0;
    int init_gg_slider_pos = GG_SLIDE_INIT;
    int target_gg_slider_pos = 0;
    int target_relic_slider_pos = 0;
    int gg_layer = 0;
    int max_gg_layer = 2;
    int [] layer_positions = {GG_SLIDE_INIT, ONE_ROTATION_40+GG_SLIDE_INIT, 2*ONE_ROTATION_40+GG_SLIDE_INIT};

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
    public DcMotor mt_lift = null;
    public DcMotor mt_test = null;

    public DcMotor mt_glyph_rotator = null;
    public DcMotor mt_glyph_slider = null;
    public DcMotor mt_intake_left = null;
    public DcMotor mt_intake_right = null;

    public Servo servoFrontLeft = null;
    public Servo servoFrontRight = null;
    public Servo servoBackLeft = null;
    public Servo servoBackRight = null;

    public Servo sv_shoulder = null;
    public Servo sv_elbow = null;
    public Servo sv_left_arm = null;
    public Servo sv_right_arm = null;
    public Servo sv_front_arm = null;
    public Servo sv_jkicker = null;

    public Servo sv_glyph_grabber_top = null;
    public Servo sv_glyph_grabber_bottom = null;

    public Servo sv_relic_grabber = null;
    public Servo sv_relic_wrist = null;
    public Servo sv_relic_elbow = null;
    // public Servo sv_test = null;
    public Servo sv_dumper = null;
    public Servo sv_dumper_gate = null;
    public Servo sv_intake_gate = null;
    public CRServo sv_bar_wheel = null;
    public ColorSensor l_colorSensor = null;
    public ColorSensor r_colorSensor = null;

    public ModernRoboticsI2cRangeSensor rangeSensorFrontRight = null;
    public ModernRoboticsI2cRangeSensor rangeSensorFrontLeft = null;
    public ModernRoboticsI2cRangeSensor rangeSensorBack = null;
    public SwerveUtilLOP.Camera camera = null;

    public SwerveUtilLOP.TeamColor leftJewelColorCamera = SwerveUtilLOP.TeamColor.UNKNOWN;
    public SwerveUtilLOP.TeamColor rightJewelColorCamera = SwerveUtilLOP.TeamColor.UNKNOWN;
    public Bitmap bitmap = null;
    public boolean camReady = false;

    public MB1202 mb_ultra = null;
    public DigitalChannel proxL = null;
    public DigitalChannel proxR = null;
    public DigitalChannel proxFL = null;
    public DigitalChannel proxML = null;


    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime runtimeAuto = new ElapsedTime();

    static double SERVO_90_DEGREE = 0.479;
    static double CRAB_DIFF_INC = 0.4663;
    static double CRAB_DIFF_DEC = 0.4762;
    static double LEFT_SV_DIFF = 0.004;
    static double RIGHT_SV_DIFF = 0.004;
    static double SERVO_FL_FORWARD_POSITION = 0.5;
    static double SERVO_FR_FORWARD_POSITION = 0.4533;
    static double SERVO_BL_FORWARD_POSITION = 0.48;
    static double SERVO_BR_FORWARD_POSITION = 0.49;

    /* variables for newbot */
    static double NB_SERVO_90_DEGREE = 0.479;
    static double NB_CRAB_DIFF_INC_FL = 0.4373;
    static double NB_CRAB_DIFF_DEC_FR = 0.4612;
    static double NB_CRAB_DIFF_INC_BL = 0.4413;
    static double NB_CRAB_DIFF_DEC_BR = 0.4872;
    static double NB_LEFT_SV_DIFF = 0.001;
    static double NB_RIGHT_SV_DIFF = 0.000;

    static double NB_SERVO_FL_FORWARD_POSITION = 0.5278;
    static double NB_SERVO_FR_FORWARD_POSITION = 0.4478;
    static double NB_SERVO_BL_FORWARD_POSITION = 0.4178;
    static double NB_SERVO_BR_FORWARD_POSITION = 0.5589;

    static double SERVO_FL_STRAFE_POSITION = SERVO_FL_FORWARD_POSITION + CRAB_DIFF_INC - LEFT_SV_DIFF;
    static double SERVO_FR_STRAFE_POSITION = SERVO_FR_FORWARD_POSITION - CRAB_DIFF_DEC + RIGHT_SV_DIFF;
    static double SERVO_BL_STRAFE_POSITION = SERVO_BL_FORWARD_POSITION + CRAB_DIFF_INC - LEFT_SV_DIFF;
    static double SERVO_BR_STRAFE_POSITION = SERVO_BR_FORWARD_POSITION - CRAB_DIFF_DEC + RIGHT_SV_DIFF;

    static double SERVO_FL_TURN_POSITION = SERVO_FL_FORWARD_POSITION - (SPOT_TURN_ANGLE_OFFSET);
    static double SERVO_FR_TURN_POSITION = SERVO_FR_FORWARD_POSITION + (SPOT_TURN_ANGLE_OFFSET);
    static double SERVO_BL_TURN_POSITION = SERVO_BL_FORWARD_POSITION + (SPOT_TURN_ANGLE_OFFSET);
    static double SERVO_BR_TURN_POSITION = SERVO_BR_FORWARD_POSITION - (SPOT_TURN_ANGLE_OFFSET);

    static double SERVO_FL_ORBIT_POSITION = SERVO_FL_FORWARD_POSITION + (THETA_FRONT / 180);
    static double SERVO_FR_ORBIT_POSITION = SERVO_FR_FORWARD_POSITION - (THETA_FRONT / 180);
    static double SERVO_BL_ORBIT_POSITION = SERVO_BL_FORWARD_POSITION + (THETA_BACK / 180);
    static double SERVO_BR_ORBIT_POSITION = SERVO_BR_FORWARD_POSITION - (THETA_BACK / 180);

    enum RelicArmPos{
        INIT, UP, FLAT, DOWN
    }
    RelicArmPos relic_arm_pos = RelicArmPos.INIT;
    enum CarMode {
        CAR,
        STRAIGHT,
        CRAB,
        TURN,
        ORBIT
    }
    CarMode cur_mode = CarMode.CAR;
    CarMode old_mode = CarMode.CAR;

    // The IMU sensor object
    BNO055IMU imu = null;
    BNO055IMU imu2 = null;
    Acceleration accel = null;

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
    public void init(HardwareMap ahwMap, Telemetry tel) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        period.reset();

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
        if (use_verbose)
            tel.addData("0: initialize Vuforia CPU time =", "%3.2f sec", period.seconds());

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

            // imu2 = hwMap.get(BNO055IMU.class, "imu2");
            // imu2.initialize(imuParameters);

            accel = imu.getAcceleration();
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        }
        if (use_verbose)
            tel.addData("0: initialize imu CPU time =", "%3.2f sec", period.seconds());

        if (use_proximity_sensor) {
            if (use_newbot) {
                proxL = hwMap.get(DigitalChannel.class, "proxL");
                proxL.setMode(DigitalChannel.Mode.INPUT);
                proxR = hwMap.get(DigitalChannel.class, "proxR");
                proxR.setMode(DigitalChannel.Mode.INPUT);
                proxFL = hwMap.get(DigitalChannel.class, "proxFL");
                proxFL.setMode(DigitalChannel.Mode.INPUT);
                proxML= hwMap.get(DigitalChannel.class, "proxML");
                proxML.setMode(DigitalChannel.Mode.INPUT);

            }
            else {
                proxL = hwMap.get(DigitalChannel.class, "prox6in");
                // set the digital channel to input.
                proxL.setMode(DigitalChannel.Mode.INPUT);
            }
        }
        if (use_color_sensor) {
            if (use_newbot) {
                if (!use_newbot_v2) {
                    l_colorSensor = hwMap.get(ColorSensor.class, "colorLeft");
                    l_colorSensor.enableLed(true);
                }
                r_colorSensor = hwMap.get(ColorSensor.class, "colorRight");
                r_colorSensor.enableLed(true);
            }
            else {
                l_colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
                l_colorSensor.enableLed(true);
                r_colorSensor = hwMap.get(ColorSensor.class, "rcolor");
                r_colorSensor.enableLed(true);
            }
        }
            if (use_range_sensor) {
                if (use_newbot) {
                    rangeSensorFrontRight = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsFrontRight");
                    rangeSensorFrontLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsFrontLeft");
                    if (use_newbot_v2) {
                        rangeSensorBack = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsBack");
                    }
                } else {
                    rangeSensorFrontRight = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsFrontRight");
                    //rangeSensorLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorLeft");
                    rangeSensorFrontLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsFrontLeft");
                }
        }
        if (use_verbose)
            tel.addData("0: initialize prox/color sensors CPU time =", "%3.2f sec", period.seconds());

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
            if (use_newbot) {
                if (use_newbot_v2) {
                    if (use_relic_elbow) {
                        sv_relic_elbow = hwMap.servo.get("sv_relic_arm");
                        sv_relic_elbow.setPosition(SV_RELIC_ELBOW_INIT);
                    }
                    sv_relic_wrist = hwMap.servo.get("sv_relic_wrist");
                    sv_relic_wrist.setPosition(SV_RELIC_WRIST_INIT);
                } else {
                    sv_relic_wrist = hwMap.servo.get("sv_relic_arm");
                    sv_relic_wrist.setPosition(SV_RELIC_WRIST_INIT);
                }
            } else {
                sv_relic_wrist = hwMap.servo.get("sv_relic_arm");
                sv_relic_wrist.setPosition(SV_RELIC_ARM_INIT);
            }
            sv_relic_grabber = hwMap.servo.get("sv_relic_grabber");
            sv_relic_grabber.setPosition((use_newbot?SV_RELIC_GRABBER_INIT_NB:SV_RELIC_GRABBER_INIT));
        }
        if (use_verbose)
            tel.addData("0: initialize relic graber CPU time =", "%3.2f sec", period.seconds());

        if (use_dumper) {
            sv_dumper = hwMap.servo.get("sv_dumper");
            sv_dumper.setPosition(SV_DUMPER_INIT);
            if (use_newbot_v2) {
                sv_intake_gate = hwMap.servo.get("sv_intake_gate");
                sv_intake_gate.setPosition(SV_INTAKE_GATE_INIT);
                if (use_dumper_gate) {
                    sv_dumper_gate = hwMap.servo.get("sv_dumper_gate");
                    sv_dumper_gate.setPosition(SV_DUMPER_GATE_INIT);
                }
                // sv_bar_wheel = hwMap.crservo.get("sv_bar_wheel");
                // sv_bar_wheel.setPower(0);
            }
            mt_lift = hwMap.dcMotor.get("mtLift");
            mt_lift.setDirection(DcMotor.Direction.REVERSE);
            mt_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mt_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (use_verbose)
            tel.addData("0: initialize dumper CPU time =", "%3.2f sec", period.seconds());

        if (use_relic_slider) {
            mt_relic_slider = hwMap.dcMotor.get("mt_relic_slider");
            if (use_newbot) {
                if (!use_newbot_v2)
                    mt_relic_slider.setDirection(DcMotor.Direction.REVERSE);
            } else {
                // mt_relic_slider.setDirection(DcMotor.Direction.REVERSE);
            }
            mt_relic_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_relic_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mt_relic_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (use_glyph_grabber) {
            sv_glyph_grabber_bottom = hwMap.servo.get("sv_grabber_bottom");
            sv_glyph_grabber_bottom.setPosition(SV_GLYPH_GRABBER_BOTTOM_INIT);

            sv_glyph_grabber_top = hwMap.servo.get("sv_grabber_top");
            sv_glyph_grabber_top.setPosition(SV_GLYPH_GRABBER_TOP_INIT);

            mt_glyph_rotator = hwMap.dcMotor.get("mt_glyph_rotator");
            mt_glyph_rotator.setDirection(DcMotor.Direction.REVERSE);
            mt_glyph_rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mt_glyph_rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mt_glyph_rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mt_glyph_slider = hwMap.dcMotor.get("mt_glyph_slider");
            // mt_glyph_slider.setDirection(DcMotor.Direction.REVERSE);
            mt_glyph_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_glyph_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mt_glyph_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (use_intake) {
            mt_intake_left = hwMap.dcMotor.get("mtIntakeLeft");
            mt_intake_left.setDirection(DcMotor.Direction.REVERSE);
            mt_intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_intake_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            mt_intake_right = hwMap.dcMotor.get("mtIntakeRight");
            // mt_glyph_slider.setDirection(DcMotor.Direction.REVERSE);
            mt_intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_intake_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        if (use_verbose)
            tel.addData("0: initialize intake CPU time =", "%3.2f sec", period.seconds());

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
        else if (use_swerve || use_newbot) {
            // Define and Initialize Motors
            motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
            motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
            if (!use_front_drive_only) {
                motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
                motorBackRight = hwMap.dcMotor.get("motorBackRight");
            }
            servoFrontRight = hwMap.servo.get("servoFrontRight");
            servoFrontLeft = hwMap.servo.get("servoFrontLeft");
            servoBackLeft = hwMap.servo.get("servoBackLeft");
            servoBackRight = hwMap.servo.get("servoBackRight");

            motorFrontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorFrontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            if (!use_front_drive_only) {
                motorBackLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
                motorBackRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            }
            // Set all motors to zero power and set all servos to central position
            // May want to change servo #'s to the value where all wheels are pointing forward.
            set_chassis_forward_position();
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            if (!use_front_drive_only) {
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
            }

            // Set all motors to run with encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (!use_front_drive_only) {
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (!use_front_drive_only) {
                motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (use_newbot) {
                initialize_newbot();
            }
        }
        if (use_verbose)
            tel.addData("0: initialize chassis CPU time =", "%3.2f sec", period.seconds());

        if (use_arm) {
            if (use_newbot) {
                if (use_newbot_v2) {
                    sv_right_arm = hwMap.servo.get("sv_right_arm");
                    sv_right_arm.setPosition(SV_RIGHT_ARM_UP_NB);
                    sv_jkicker = hwMap.servo.get("sv_jkicker");
                    sv_jkicker.setPosition(SV_JKICKER_INIT);
                }
                else {
                    sv_left_arm = hwMap.servo.get("sv_left_arm");
                    sv_right_arm = hwMap.servo.get("sv_right_arm");
                    sv_left_arm.setPosition(SV_LEFT_ARM_UP_NB);
                    sv_right_arm.setPosition(SV_RIGHT_ARM_UP_NB);
                }
            }
            else{
                sv_elbow = hwMap.servo.get("sv_elbow");
                sv_shoulder = hwMap.servo.get("sv_shoulder");
                sv_right_arm = hwMap.servo.get("sv_right_arm");
                sv_elbow.setPosition(SV_ELBOW_UP);
                sv_shoulder.setPosition(SV_SHOULDER_INIT);
                sv_right_arm.setPosition(SV_RIGHT_ARM_UP);
            }

        }
        if (use_front_arm) {
            sv_front_arm = hwMap.servo.get("sv_front_arm");
            sv_front_arm.setPosition(SV_FRONT_ARM_IN);
        }
        if (use_verbose)
            tel.addData("0: initialize arms CPU time =", "%3.2f sec", period.seconds());

    }

    void set_chassis_forward_position() {
        if (use_newbot) {
            servoFrontLeft.setPosition(NB_SERVO_FL_FORWARD_POSITION);
            servoFrontRight.setPosition(NB_SERVO_FR_FORWARD_POSITION);
            servoBackLeft.setPosition(NB_SERVO_BL_FORWARD_POSITION);
            servoBackRight.setPosition(NB_SERVO_BR_FORWARD_POSITION);

            servoPosFL = NB_SERVO_FL_FORWARD_POSITION;
            servoPosFR = NB_SERVO_FR_FORWARD_POSITION;
            servoPosBL = NB_SERVO_BL_FORWARD_POSITION;
            servoPosBR = NB_SERVO_BR_FORWARD_POSITION;
        } else {
            servoFrontLeft.setPosition(SERVO_FL_FORWARD_POSITION);
            servoFrontRight.setPosition(SERVO_FR_FORWARD_POSITION);
            servoBackLeft.setPosition(SERVO_BL_FORWARD_POSITION);
            servoBackRight.setPosition(SERVO_BR_FORWARD_POSITION);

            servoPosFL = SERVO_FL_FORWARD_POSITION;
            servoPosFR = SERVO_FR_FORWARD_POSITION;
            servoPosBL = SERVO_BL_FORWARD_POSITION;
            servoPosBR = SERVO_BR_FORWARD_POSITION;
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

    void initialize_newbot() {
        SPOT_TURN_ANGLE_OFFSET = (Math.atan((0.5*NB_LENGTH_BETWEEN_WHEELS)/(0.5*NB_WIDTH_BETWEEN_WHEELS))) * (1/(Math.PI));

        SERVO_FL_STRAFE_POSITION = NB_SERVO_FL_FORWARD_POSITION + NB_CRAB_DIFF_INC_FL - NB_LEFT_SV_DIFF;
        if (SERVO_FL_STRAFE_POSITION>1.0)
            SERVO_FL_STRAFE_POSITION = 1.0;
        SERVO_FR_STRAFE_POSITION = NB_SERVO_FR_FORWARD_POSITION - NB_CRAB_DIFF_DEC_FR + NB_RIGHT_SV_DIFF;
        if (SERVO_FR_STRAFE_POSITION<0.0)
            SERVO_FR_STRAFE_POSITION = 0.0;
        SERVO_BL_STRAFE_POSITION = NB_SERVO_BL_FORWARD_POSITION + NB_CRAB_DIFF_INC_BL - NB_LEFT_SV_DIFF;
        if (SERVO_BL_STRAFE_POSITION>1.0)
            SERVO_BL_STRAFE_POSITION = 1.0;
        SERVO_BR_STRAFE_POSITION = NB_SERVO_BR_FORWARD_POSITION - NB_CRAB_DIFF_DEC_BR + NB_RIGHT_SV_DIFF;
        if (SERVO_BR_STRAFE_POSITION<0.0)
            SERVO_BR_STRAFE_POSITION = 0.0;
        double NB_SERVO_UNIT_CONVERSION = Math.abs(SERVO_FL_STRAFE_POSITION - SERVO_FL_FORWARD_POSITION)/0.5;

        SERVO_FL_TURN_POSITION = NB_SERVO_FL_FORWARD_POSITION  - (NB_SPOT_TURN_ANGLE_OFFSET * NB_SERVO_UNIT_CONVERSION);
        SERVO_FR_TURN_POSITION = NB_SERVO_FR_FORWARD_POSITION  + (NB_SPOT_TURN_ANGLE_OFFSET * NB_SERVO_UNIT_CONVERSION);
        SERVO_BL_TURN_POSITION = NB_SERVO_BL_FORWARD_POSITION  + (NB_SPOT_TURN_ANGLE_OFFSET * NB_SERVO_UNIT_CONVERSION);
        SERVO_BR_TURN_POSITION = NB_SERVO_BR_FORWARD_POSITION  - (NB_SPOT_TURN_ANGLE_OFFSET * NB_SERVO_UNIT_CONVERSION);

        SERVO_FL_ORBIT_POSITION = NB_SERVO_FL_FORWARD_POSITION + (NB_THETA_FRONT / 180 * NB_SERVO_UNIT_CONVERSION);
        SERVO_FR_ORBIT_POSITION = NB_SERVO_FR_FORWARD_POSITION - (NB_THETA_FRONT / 180 * NB_SERVO_UNIT_CONVERSION);
        SERVO_BL_ORBIT_POSITION = NB_SERVO_BL_FORWARD_POSITION + (NB_THETA_BACK / 180 * NB_SERVO_UNIT_CONVERSION);
        SERVO_BR_ORBIT_POSITION = NB_SERVO_BR_FORWARD_POSITION - (NB_THETA_BACK / 180 * NB_SERVO_UNIT_CONVERSION);
    }
}
