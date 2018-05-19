package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SwerveDriveHardware;
import org.firstinspires.ftc.teamcode.SwerveUtilLOP;
import org.firstinspires.ftc.teamcode.template.hardware.CoreSystem;

/**
 * Holding class for methods that access objects in ways that should be disallowed.
 */
public class TaintedAccess {

    private SystemControl systemControl;

    TaintedAccess(SystemControl systemControl) {
        this.systemControl = systemControl;
    }

    SwerveUtilLOP getSwerveUtilLOP() {
        if (this.systemControl.swerveUtilLOP == null) {
            throw new IllegalStateException ("Internal TaintedAccess.swerveUtilLOP object must be set. ");
        }
        return this.systemControl.swerveUtilLOP;
    }

    void stop_chassis() {
        getSwerveUtilLOP().stop_chassis();
    }

    void intakeGateInit() {
        getSwerveUtilLOP().intakeGateInit();
    }

}
