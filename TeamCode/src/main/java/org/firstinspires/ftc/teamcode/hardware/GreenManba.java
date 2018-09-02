package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SwerveUtilLOP;

/**
 * Main container class for this robot's subsystems.
 * Its methods influence how the entire robot operates
 */
public class GreenManba {

    public CoreSystem coreSystem;

    public RelicReachSystem relicReachSystem;
    public SwerveSystem swerve;
    public GlyphIntakeSystem intake;
    public GlyphDumperSystem dumper;
    public JewelSystem jewel;
    public CameraSystem camera;

    // define all switches to turn on/off hardware each component
    public boolean use_verbose = false;
    public boolean servo_tune_up = false;

    public SwerveUtilLOP.TeamColor allianceColor = SwerveUtilLOP.TeamColor.BLUE; // default blue team

    public boolean isTesting = false;  // [Moved to TeleOp as class variable]
    public boolean gg_slider_encoder_ok = false; // used by glyph lifter
    public boolean stop_on_bump = false; // [autonomous variable]
    public boolean bump_detected = false; // [autonomous variable]
    public boolean tried_clockwise = false;
    public boolean snaked_left = false;

    public int targetColumn = -1; // [autonomous variable]

    public ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime runtimeAuto = new ElapsedTime();

    final static int ONE_ROTATION_60 = 1680; // AndyMark NeveRest-60
    final static int ONE_ROTATION_40 = 1120; // AndyMark NeveRest-40
    final static int ONE_ROTATION_20 = 560; // AndyMark NeveRest-20
    final static int ONE_ROTATION = 538; // for new AndyMark-20 motor encoder one rotation


    /* local OpMode members. */
    HardwareMap hwMap = null; // [ convert to local variable]
    private ElapsedTime period = new ElapsedTime();

    /**
     * Create a SystemControl object.
     */
    public GreenManba() {
        this.coreSystem = new CoreSystem();
        this.relicReachSystem = new RelicReachSystem(this.coreSystem);
        this.swerve = new SwerveSystem(this.coreSystem);
        this.intake = new GlyphIntakeSystem(this.coreSystem);
        this.dumper = new GlyphDumperSystem(this.coreSystem);
        this.jewel = new JewelSystem(this.coreSystem);
        this.camera = new CameraSystem(this.coreSystem);
    }

    /**
     * Initialize the robot's central control and subsystems
     *
     * It takes a supplierForCanContinue object that essentially drives the behavior of the core's
     * 'canContinue method (see {@link CoreSystem}).
     * It This object is queried when ceasing the current operation would not be inconvenient. During any long lapses in which it
     * is not invoked, there is a risk that the robot will appear unresponsive until it finally checks and reacts.
     *
     * @param map  map of hardware devices
     * @param supplierForCanContinue supplier for 'canContinue' requests
     *
     */
    public void init(HardwareMap map, Supplier<Boolean> supplierForCanContinue) {
        if (supplierForCanContinue == null) {
            throw new AssertionError("supplierForCanContinue cannot be null");
        }
        this.coreSystem.supplierForCanContinue = supplierForCanContinue;
        this.relicReachSystem.init(map);
        this.swerve.init(map);
        this.intake.init(map);
        this.dumper.init(map);
        this.jewel.init(map);
        this.camera.init(map);
        if (use_verbose) {
            coreSystem.telemetry.addData("0: initialize hardware, CPU time =", "%3.2f sec", period.seconds());
            coreSystem.telemetry.update();
        }
    }

    //
    // Start of 'tainted access support' section
    // TODO: Remove this section when these (undesirable) dependencies are no longer necessary

    TaintedAccess taintedAccess;
    // SwerveUtilLOP swerveUtilLOP;

    /**
     * Initialize 'tainted access' across subsystems
     * @param swerveUtilLOP legacy utility code object
     */
    public void initTaintedAccess(SwerveUtilLOP swerveUtilLOP) {
        //this.swerveUtilLOP = swerveUtilLOP;
        this.taintedAccess = new TaintedAccess(this);
        this.relicReachSystem.setTaintedAccess(this.taintedAccess);
    }

    // ... end of 'tainted access support'

}
