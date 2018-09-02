package org.firstinspires.ftc.teamcode;

//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.hardware.CameraSystem;
import org.firstinspires.ftc.teamcode.hardware.GlyphDumperSystem;
import org.firstinspires.ftc.teamcode.hardware.GlyphIntakeSystem;
import org.firstinspires.ftc.teamcode.hardware.JewelSystem;
import org.firstinspires.ftc.teamcode.hardware.RelicReachSystem;
import org.firstinspires.ftc.teamcode.hardware.SwerveSystem;
import org.firstinspires.ftc.teamcode.hardware.GreenManba;

import static java.lang.Thread.sleep;

public class SwerveDriveHardware {

    /**
     * Central container for robotic subsystems classes
     */
    final public GreenManba greenManba;

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
    public boolean servo_tune_up = false;

    public SwerveUtilLOP.TeamColor allianceColor = SwerveUtilLOP.TeamColor.BLUE; // default blue team

    boolean isTesting = false;  // [Moved to TeleOp as class variable]
    boolean gg_slider_encoder_ok = false; // used by glyph lifter
    boolean stop_on_bump = false; // [autonomous variable]
    boolean bump_detected = false; // [autonomous variable]

    int targetColumn = -1; // [autonomous variable]

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
        this.greenManba = new GreenManba();
        this.relicReachSystem = greenManba.relicReachSystem;
        this.swerve = greenManba.swerve;
        this.intake = greenManba.intake;
        this.dumper = greenManba.dumper;
        this.jewel = greenManba.jewel;
        this.camera = greenManba.camera;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry tel, Supplier<Boolean> supplierForOpModeIsActive, SwerveUtilLOP swerveUtilLOP) {
        this.greenManba.coreSystem.telemetry = tel;
        this.greenManba.initTaintedAccess(swerveUtilLOP);
        // Save reference to Hardware map
        hwMap = ahwMap;
        period.reset();

        this.greenManba.init(hwMap, supplierForOpModeIsActive);
        if (use_verbose) {
            tel.addData("0: initialize hardware, CPU time =", "%3.2f sec", period.seconds());
            tel.update();
        }
    }

}
