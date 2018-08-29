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
import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.hardware.RelicReachSystem;
import org.firstinspires.ftc.teamcode.hardware.SwerveSystem;
import org.firstinspires.ftc.teamcode.hardware.SystemControl;

import static java.lang.Thread.sleep;

public class SwerveDriveHardware {

    /**
     * Central container for robotic subsystems classes
     */
    final private SystemControl systemControl;

    /**
     * Relic reach subsystem
     */
    final public RelicReachSystem relicReachSystem;
    final public SwerveSystem swerve;

    // define all switches to turn on/off hardware each component
    public boolean use_verbose = false;
    public boolean use_intake = false;
    public boolean use_dumper = false;
    public boolean use_dumper_gate = true;
    public boolean use_Vuforia = true;
    public boolean use_camera = false;
    public boolean use_color_sensor = false;
    public boolean use_range_sensor = false;
    public boolean use_glyph_grabber = false;
    public boolean use_arm = false;
    public boolean use_proximity_sensor = false;
    //public boolean use_front_arm = false;
    public boolean servo_tune_up = false;

    public SwerveUtilLOP.TeamColor allianceColor = SwerveUtilLOP.TeamColor.BLUE; // default blue team

    boolean isTesting = false;  // [Moved to TeleOp as class variable]
    boolean gg_slider_encoder_ok = false; // [glyph dump system ?]
    boolean gg_rotator_encoder_ok = false;
    boolean gg_top_close = false;
    boolean gg_bottom_close = false;
    boolean is_gg_upside_down = false;
    boolean stop_on_bump = false; // [autonomous variable]
    boolean bump_detected = false; // [autonomous variable]

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
    final static double SV_RIGHT_ARM_UP_NB = 0.02;
    final static double SV_RIGHT_ARM_DOWN_NB = 0.43;
    final static double SV_LEFT_ARM_UP_NB = 0.943;
    final static double SV_LEFT_ARM_DOWN_NB = 0.359;
    final static double SV_FRONT_ARM_IN = 0.83;
    final static double SV_FRONT_ARM_OUT = 0.43;
    final static double SV_JKICKER_UP = 0.47;
    final static double SV_JKICKER_RIGHT1 = 0.57;
    final static double SV_JKICKER_RIGHT2 = 0.65;
    final static double SV_JKICKER_RIGHT = 0.85;
    final static double SV_JKICKER_LEFT1 = 0.37;
    final static double SV_JKICKER_LEFT2 = 0.27;
    final static double SV_JKICKER_LEFT = 0.1;
    final static double SV_JKICKER_INIT = 0.82;

    final static double SV_DUMPER_INIT = 0.6822;
    final static double SV_DUMPER_DOWN = 0.6822;
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
    double intakeRatio = 1.0;
    int targetColumn = -1; // [autonomous variable]

    double blue = 0; // [convert to local]
    double red = 0; // [convert to local ]

    int orig_rot_pos = 0;
    int target_rot_pos = 0;
    int init_gg_slider_pos = GG_SLIDE_INIT;
    int target_gg_slider_pos = 0;
    int target_relic_slider_pos = 0;
    int gg_layer = 0;
    int max_gg_layer = 2;
    int [] layer_positions = {GG_SLIDE_INIT, ONE_ROTATION_40+GG_SLIDE_INIT, 2*ONE_ROTATION_40+GG_SLIDE_INIT};

    /* Public OpMode members. */
    public DcMotor mt_lift = null;
    public DcMotor mt_test = null;

    //    public DcMotor mt_glyph_rotator = null;
    public DcMotor mt_glyph_slider = null;
    public DcMotor mt_intake_left = null;
    public DcMotor mt_intake_right = null;

    public Servo sv_shoulder = null;
    public Servo sv_elbow = null;
    public Servo sv_left_arm = null;
    public Servo sv_right_arm = null;
    //    public Servo sv_front_arm = null;
    public Servo sv_jkicker = null;

//    public Servo sv_glyph_grabber_top = null;
//    public Servo sv_glyph_grabber_bottom = null;

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

    final static int ONE_ROTATION_60 = 1680; // AndyMark NeveRest-60
    final static int ONE_ROTATION_40 = 1120; // AndyMark NeveRest-40
    final static int ONE_ROTATION_20 = 560; // AndyMark NeveRest-20
    final static int ONE_ROTATION = 538; // for new AndyMark-20 motor encoder one rotation


    // The IMU sensor object
    //BNO055IMU imu = null;
    //BNO055IMU imu2 = null;
    //Acceleration accel = null;

    // State used for updating telemetry
    //Orientation angles; // [ convert to local variable]
    //Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap = null; // [ convert to local variable]
    private ElapsedTime period = new ElapsedTime();

    VuforiaLocalizer vuforia;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    VuforiaLocalizer.Parameters parameters;

    /* Constructor */
    public SwerveDriveHardware() {
        this.systemControl = new SystemControl();
        this.relicReachSystem = systemControl.relicReachSystem;
        this.swerve = systemControl.swerve;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry tel, Supplier<Boolean> supplierForOpModeIsActive, SwerveUtilLOP swerveUtilLOP) {
        this.systemControl.initTaintedAccess(swerveUtilLOP);
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

        if (use_proximity_sensor) {
            if (swerve.use_newbot) {
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
            if (swerve.use_newbot) {
                if (!swerve.use_newbot_v2) {
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
                if (swerve.use_newbot) {
                    rangeSensorFrontRight = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsFrontRight");
                    rangeSensorFrontLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rsFrontLeft");
                    if (swerve.use_newbot_v2) {
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

        if (use_dumper) {
            sv_dumper = hwMap.servo.get("sv_dumper");
            sv_dumper.setPosition(SV_DUMPER_INIT);
            if (swerve.use_newbot_v2) {
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

        this.systemControl.init(hwMap, tel, period, supplierForOpModeIsActive);
        if (use_verbose)
            tel.addData("0: initialize relic graber CPU time =", "%3.2f sec", period.seconds());

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

        if (use_arm) {
            if (swerve.use_newbot) {
                if (swerve.use_newbot_v2) {
                    sv_right_arm = hwMap.servo.get("sv_right_arm");
                    sv_right_arm.setPosition(SV_RIGHT_ARM_UP_NB);
                    // sv_jkicker = hwMap.servo.get("sv_jkicker");
                    // sv_jkicker.setPosition(SV_JKICKER_INIT);
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
//        if (use_front_arm) {
//            sv_front_arm = hwMap.servo.get("sv_front_arm");
//            sv_front_arm.setPosition(SV_FRONT_ARM_IN);
//        }
        if (use_verbose)
            tel.addData("0: initialize arms CPU time =", "%3.2f sec", period.seconds());

    }

}
