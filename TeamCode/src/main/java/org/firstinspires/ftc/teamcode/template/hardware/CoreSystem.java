package org.firstinspires.ftc.teamcode.template.hardware;

import org.firstinspires.ftc.robotcore.external.Supplier;

/**
 * Central system for the robot, which all of the other subsystems can access.
 */

public class CoreSystem {

    final static long MAX_NAP_LENGTH = 500; // Maximum internal sleep segment (seconds)

    Supplier<Boolean> supplierForCanContinue;

    /**
     * Indicates if robot is allowed to continue proceed with any operation
     * @return false if the operation must halt; true otherwise
     */
    public boolean canContinue() {
        Boolean result = this.supplierForCanContinue.get();
        return result;
    }

    /**
     * Cease processing and for a specified length  of time (seconds).
     * @param duration  length of asleep (seconds)
     */
    public void sleep(double duration) {
        long length = (long) (1000L * duration);
        while (length > 0) {
            if (!this.canContinue()) {
                return;
            }
            long napLength = MAX_NAP_LENGTH;
            if (napLength > length) {
                napLength = length;
            }
            try {
                Thread.sleep(napLength);
            } catch (InterruptedException e) {
                return;
            }
            length -= napLength;
        }
    }


}
