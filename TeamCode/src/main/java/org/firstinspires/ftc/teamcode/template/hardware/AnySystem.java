package org.firstinspires.ftc.teamcode.template.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Put brief class description here...
 */
public class AnySystem {

    // put class constants and class variables

    // TODO: include each hardware device
    // DCMotor xyzMotor;
    // etc.

    // Central core of robot
    CoreSystem core;

    AnySystem(CoreSystem core) {
        this.core = core;
    }

    // Initialize hardware devices and other settings
    void init(HardwareMap map) {
        // TODO: initialize each hardware device
        // this.xyzMotor = map.get("xyz_motor");
        // this.xyzMotor.setDirection(Direction.FORWARD);
        // etc.
    }

    // put class methods here...

}
