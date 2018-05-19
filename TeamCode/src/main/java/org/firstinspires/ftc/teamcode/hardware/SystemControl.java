package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.SwerveUtilLOP;

/**
 * Main container class for this robot's subsystems.
 * Its methods influence how the entire robot operates
 */
public class SystemControl {

    public CoreSystem coreSystem;

    public RelicReachSystem relicReachSystem;

    /**
     * Create a SystemControl object.
     */
    public SystemControl() {
        this.coreSystem = new CoreSystem();

        this.relicReachSystem = new RelicReachSystem(this.coreSystem);

    }

    /**
     * Initialize the robot's central control and subsystems
     *
     * It takes a supplierForCanContinue object that essentially drives the behavior of the core's
     * 'canContinue method (see {@link CoreSystem}).
     * It This object is queried when ceasing the current operation would not be inconvenient. During any long lapses in which it
     * is not invoked, there is a risk that the robot will appear unresponsive until it finally checks and reacts.
     *
     * @param map  map of hardware devicess
     * @param supplierForCanContinue supplier for 'canContinue' requests
     *
     */
    public void init(HardwareMap map, Supplier<Boolean> supplierForCanContinue) {
        if (supplierForCanContinue == null) {
            throw new AssertionError("supplierForCanContinue cannot be null");
        }
        this.coreSystem.supplierForCanContinue = supplierForCanContinue;
        this.relicReachSystem.init(map);
    }

    //
    // Start of 'tainted access support' section
    // TODO: Remove this section when these (undesireable) dependencies are no longer necessary

    TaintedAccess taintedAccess;
    SwerveUtilLOP swerveUtilLOP;

    /**
     * Initialize 'tainted access' across subsystems
     * @param swerveUtilLOP legacy utility code object
     */
    public void initTaintedAccess(SwerveUtilLOP swerveUtilLOP) {
        this.swerveUtilLOP = swerveUtilLOP;
        this.taintedAccess = new TaintedAccess(this);
        this.relicReachSystem.setTaintedAccess(this.taintedAccess);
    }

    // ... end of 'tainted access support'

}
