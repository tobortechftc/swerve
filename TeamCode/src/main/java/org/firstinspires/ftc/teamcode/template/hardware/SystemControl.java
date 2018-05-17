package org.firstinspires.ftc.teamcode.template.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;

/**
 * Main container class for this robot's subsystems.
 * Its methods influence how the entire robot operates
 */
public class SystemControl {

    public CoreSystem core;

    // TODO: declare each subsystem
    // public AnySystem any;
    // etc.

    /**
     * Create a SystemControl object.
     */
    public SystemControl() {
        this.core = new CoreSystem();

        // TODO: populate each subsystem
        // this.any = new AnySystem(this.core);
        // etc.

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
        this.core.supplierForCanContinue = supplierForCanContinue;

        // TODO: initialize each subsystem
        // this.any.init(map);
        // etc.
    }

}
