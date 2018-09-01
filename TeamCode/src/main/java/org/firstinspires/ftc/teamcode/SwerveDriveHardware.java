package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

//import com.qualcomm.ftccommon.DbgLog;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.hardware.CameraSystem;
import org.firstinspires.ftc.teamcode.hardware.GlyphDumperSystem;
import org.firstinspires.ftc.teamcode.hardware.GlyphIntakeSystem;
import org.firstinspires.ftc.teamcode.hardware.JewelSystem;
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
    final public GlyphIntakeSystem intake;
    final public GlyphDumperSystem dumper;
    final public JewelSystem jewel;
    final public CameraSystem camera;


    // define all switches to turn on/off hardware each component
    public boolean use_verbose = false;
    public boolean use_range_sensor = false;
    public boolean use_proximity_sensor = false;
    //public boolean use_front_arm = false;
    public boolean servo_tune_up = false;

    public SwerveUtilLOP.TeamColor allianceColor = SwerveUtilLOP.TeamColor.BLUE; // default blue team

    boolean isTesting = false;  // [Moved to TeleOp as class variable]
    boolean gg_slider_encoder_ok = false; // used by glyph lifter
    boolean stop_on_bump = false; // [autonomous variable]
    boolean bump_detected = false; // [autonomous variable]

    int targetColumn = -1; // [autonomous variable]

    public ModernRoboticsI2cRangeSensor rangeSensorFrontRight = null;
    public ModernRoboticsI2cRangeSensor rangeSensorFrontLeft = null;
    public ModernRoboticsI2cRangeSensor rangeSensorBack = null;
    // public SwerveUtilLOP.Camera icamera = null;

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


    /* local OpMode members. */
    HardwareMap hwMap = null; // [ convert to local variable]
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public SwerveDriveHardware() {
        this.systemControl = new SystemControl();
        this.relicReachSystem = systemControl.relicReachSystem;
        this.swerve = systemControl.swerve;
        this.intake = systemControl.intake;
        this.dumper = systemControl.dumper;
        this.jewel = systemControl.jewel;
        this.camera = systemControl.camera;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry tel, Supplier<Boolean> supplierForOpModeIsActive, SwerveUtilLOP swerveUtilLOP) {
        this.systemControl.initTaintedAccess(swerveUtilLOP);
        // Save reference to Hardware map
        hwMap = ahwMap;
        period.reset();

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
        if (use_verbose) {
            tel.addData("0: initialize prox/color sensors CPU time =", "%3.2f sec", period.seconds());
            tel.update();
        }

        this.systemControl.init(hwMap, tel, period, supplierForOpModeIsActive);
        if (use_verbose) {
            tel.addData("0: initialize hardware, CPU time =", "%3.2f sec", period.seconds());
            tel.update();
        }
    }

}
