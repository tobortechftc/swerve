package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.template.hardware.CoreSystem;

/**
 * Put brief class description here...
 */
public class SharedSystem {

    public boolean use_verbose = false;

    private SystemControl systemControl;  // TODO: Get rid of this line!! (it's kluge code)

    SharedSystem(SystemControl systemControl) {
        this.systemControl = systemControl;
    }

    // put class methods here...

    void stop_chassis() {
        // TODO: implement code
        throw new IllegalStateException("stopChassis not implemented");
    }

    void intakeGateInit() {
        // TODO: implement code
        throw new IllegalStateException("intakeGateInit not implemented");
    }

}
