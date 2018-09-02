package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.SwerveUtilLOP;

/**
 * Holding class for methods that access objects in ways that should be disallowed.
 */
public class TaintedAccess {

    private GreenManba greenManba;

    TaintedAccess(GreenManba greenManba) {
        this.greenManba = greenManba;
    }

//    SwerveUtilLOP getSwerveUtilLOP() {
//        if (this.greenManba.swerveUtilLOP == null) {
//            throw new IllegalStateException ("Internal TaintedAccess.swerveUtilLOP object must be set. ");
//        }
//        return this.greenManba.swerveUtilLOP;
//    }

    void stop_chassis() {
        greenManba.swerve.stop_chassis();
    }

    void intakeGateInit() {
        greenManba.intake.intakeGateInit();
    }

}
