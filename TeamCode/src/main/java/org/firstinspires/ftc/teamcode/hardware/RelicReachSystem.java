package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Put brief class description here...
 */
public class RelicReachSystem {

    final static double SV_RELIC_GRABBER_INIT = 0.5039;
    final static double SV_RELIC_GRABBER_CLOSE = 0.5006;
    final static double SV_RELIC_GRABBER_OPEN = 0.1822;
    final static double SV_RELIC_GRABBER_INIT_NB = 0.86;
    final static double SV_RELIC_GRABBER_CLOSE_NB = 0.8;
    final static double SV_RELIC_GRABBER_OPEN_NB = 0.25;
    final static double SV_RELIC_GRABBER_OPEN_W_NB = 0.15;
    final static double SV_RELIC_ARM_INIT = 0.1;
    final static double SV_RELIC_ARM_UP = 0.7;
    final static double SV_RELIC_ARM_MIDDLE = 0.55;
    final static double SV_RELIC_ARM_DOWN = 0.2;
    final static double SV_RELIC_ARM_DOWN_R = 0.27; // down and ready for release
    final static double SV_RELIC_WRIST_INIT = 0.329;
    final static double SV_RELIC_WRIST_UP = 0.99;
    final static double SV_RELIC_WRIST_MIDDLE = 0.78;
    public final static double SV_RELIC_WRIST_DOWN = 0.379;
    final static double SV_RELIC_WRIST_DOWN_R = 0.4; // down and ready for release
    final static double SV_RELIC_WRIST_DOWN_AUTO = SV_RELIC_WRIST_DOWN;

    final static double SV_RELIC_ELBOW_INIT = 0.6167;
    final static double SV_RELIC_ELBOW_UP = 0.5689;
    final static double SV_RELIC_ELBOW_FLAT = 0.5117;
    final static double SV_RELIC_ELBOW_DOWN = 0.5;
    final static int RELIC_SLIDE_MAX = -8800;

    public boolean use_verbose = false;
    public boolean use_relic_elbow = false;
    public boolean use_relic_grabber = true;
    public boolean use_relic_slider = true;

    public Servo sv_relic_grabber = null;
    public Servo sv_relic_wrist = null;
    public Servo sv_relic_elbow = null;
    public DcMotor mt_relic_slider = null;

    enum RelicArmPos{
        INIT, UP, FLAT, DOWN
    }

    RelicArmPos relic_arm_pos = RelicArmPos.INIT;

    // Central core of robot
    CoreSystem core;

    private TaintedAccess taintedAccess;
    void setTaintedAccess(TaintedAccess taintedAccess) {
        this.taintedAccess = taintedAccess;
    }

    RelicReachSystem(CoreSystem core) {
        this.core = core;
    }

    public void enable(boolean isAuto) {
        if (isAuto) {
            use_relic_elbow = false;
            use_relic_grabber = false;
            use_relic_slider = false;
        } else {
            use_relic_elbow = false;
            use_relic_grabber = true;
            use_relic_slider = true;
        }
    }

    public void disable() {
        use_relic_elbow = false;
        use_relic_grabber = false;
        use_relic_slider = false;
    }
    void init(HardwareMap hwMap) {
        if (use_relic_grabber) {
                    if (use_relic_elbow) {
                        sv_relic_elbow = hwMap.servo.get("sv_relic_arm");
                        sv_relic_elbow.setPosition(SV_RELIC_ELBOW_INIT);
                    }
                    sv_relic_wrist = hwMap.servo.get("sv_relic_wrist");
                    sv_relic_wrist.setPosition(SV_RELIC_WRIST_INIT);
            sv_relic_grabber = hwMap.servo.get("sv_relic_grabber");
            sv_relic_grabber.setPosition(SV_RELIC_GRABBER_INIT_NB);
        }
        if (use_relic_slider) {
            mt_relic_slider = hwMap.dcMotor.get("mt_relic_slider");
            // mt_relic_slider.setDirection(DcMotor.Direction.REVERSE);
            mt_relic_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_relic_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mt_relic_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (use_verbose) {
            core.telemetry.addData("0: initialize Relic system CPU time =", "%3.2f sec", core.run_seconds());
            core.telemetry.update();
        }
    }

    public void relic_grabber_lower() {
        double pos = sv_relic_grabber.getPosition()-0.04;
        if (pos<0.001) pos = 0.001;
        sv_relic_grabber.setPosition(pos);
    }

    public void relic_grabber_higher() {
        double pos = sv_relic_grabber.getPosition()+0.04;
        if (pos>0.99) pos = 0.99;
        sv_relic_grabber.setPosition(pos);
    }

    public void relic_grabber_close() {
        sv_relic_grabber.setPosition(SV_RELIC_GRABBER_CLOSE_NB);
    }

    public void relic_grabber_open(boolean wide) {
            if (wide) {
                sv_relic_grabber.setPosition(SV_RELIC_GRABBER_OPEN_W_NB);
            } else {
                sv_relic_grabber.setPosition(SV_RELIC_GRABBER_OPEN_NB);
            }
    }

    public void relic_grabber_release() {
        // sv_relic_wrist.setPosition(SV_RELIC_ARM_DOWN_R);
        double pos = sv_relic_grabber.getPosition();
        double tar = SV_RELIC_GRABBER_OPEN_NB;
        if (pos < tar) {
            sv_relic_grabber.setPosition(pos+0.1);
            core.sleep(.250);
            sv_relic_grabber.setPosition(pos+0.2);
            core.sleep(.250);
        } else {
            sv_relic_grabber.setPosition(pos-0.1);
            core.sleep(.250);
            sv_relic_grabber.setPosition(pos-0.2);
            core.sleep(.250);
        }
        sv_relic_grabber.setPosition(tar);
    }

    public void relic_slider_in(double power, boolean force) {
        if (Math.abs(power)>1) power=1;
        double pos = mt_relic_slider.getCurrentPosition();
        if (pos>-100 && power!=0 && force==false) {
            if (pos<0) power = 0.1;
            else {
                power = 0;
                mt_relic_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mt_relic_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        if (power>0) taintedAccess.stop_chassis();
        mt_relic_slider.setPower(Math.abs(power));
    }

    public void relic_slider_out(double power) {
        if (Math.abs(power)>1) power=1;
        if (power>0) taintedAccess.stop_chassis();
        mt_relic_slider.setPower(-1*Math.abs(power));
    }

    public void relic_slider_stop() {
        mt_relic_slider.setPower(0);
    }

    public void relic_elbow_up_auto() {
        // down -> flat -> up -> init
        if (!use_relic_elbow) return;
        if (relic_arm_pos == RelicReachSystem.RelicArmPos.DOWN) {
            relic_elbow_flat();
            relic_arm_ready_grab_and_release();
        } else if (relic_arm_pos == RelicReachSystem.RelicArmPos.FLAT) {
            // relic_elbow_up();
            relic_arm_ready_delivery();
        } else {
            relic_elbow_init();
        }
    }

    public void relic_elbow_down_auto() {
        // init -> up -> flat -> down
        if (!use_relic_elbow) return;
        if (relic_arm_pos == RelicReachSystem.RelicArmPos.INIT) {
            relic_elbow_up();
        } else if (relic_arm_pos == RelicReachSystem.RelicArmPos.UP) {
            // relic_elbow_flat();
            relic_arm_ready_grab_and_release();
        } else {
            relic_elbow_down();
        }
    }

    public void relic_elbow_higher() {
        if (!use_relic_elbow) return;
        double new_pos = sv_relic_elbow.getPosition() + 0.001;
        if (new_pos>0.999) new_pos = 0.999;
        sv_relic_elbow.setPosition(new_pos);
        if (new_pos>=SV_RELIC_ELBOW_INIT)
            relic_arm_pos = RelicReachSystem.RelicArmPos.INIT;
        else if (new_pos>=SV_RELIC_ELBOW_UP)
            relic_arm_pos = RelicReachSystem.RelicArmPos.UP;
        else if (new_pos>=SV_RELIC_ELBOW_FLAT)
            relic_arm_pos = RelicReachSystem.RelicArmPos.FLAT;
        else
            relic_arm_pos = RelicReachSystem.RelicArmPos.DOWN;
    }

    public void relic_elbow_lower() {
        if (!use_relic_elbow) return;
        double new_pos = sv_relic_elbow.getPosition() - 0.001;
        if (new_pos<0.001) new_pos = 0.001;
        sv_relic_elbow.setPosition(new_pos);
        if (new_pos<=SV_RELIC_ELBOW_DOWN)
            relic_arm_pos = RelicReachSystem.RelicArmPos.DOWN;
        else if (new_pos<=SV_RELIC_ELBOW_FLAT)
            relic_arm_pos = RelicReachSystem.RelicArmPos.FLAT;
        else if (new_pos<=SV_RELIC_ELBOW_UP)
            relic_arm_pos = RelicReachSystem.RelicArmPos.UP;
        else
            relic_arm_pos = RelicReachSystem.RelicArmPos.INIT;
    }

    public void relic_elbow_up() {
        if (!use_relic_elbow) return;
        sv_relic_elbow.setPosition(SV_RELIC_ELBOW_UP);
        relic_arm_pos = RelicReachSystem.RelicArmPos.UP;
    }

    public void relic_elbow_flat() {
        if (!use_relic_elbow) return;
        sv_relic_elbow.setPosition(SV_RELIC_ELBOW_FLAT);
        relic_arm_pos = RelicReachSystem.RelicArmPos.FLAT;
    }

    public void relic_elbow_down() {
        if (!use_relic_elbow) return;
        sv_relic_elbow.setPosition(SV_RELIC_ELBOW_DOWN);
        relic_arm_pos = RelicReachSystem.RelicArmPos.DOWN;
    }

    public void relic_elbow_init() {
        if (!use_relic_elbow) return;
        sv_relic_elbow.setPosition(SV_RELIC_ELBOW_INIT);
        relic_arm_pos = RelicReachSystem.RelicArmPos.INIT;
    }

    public void relic_arm_ready_grab_and_release() {
        if (!use_relic_elbow) return;
        relic_elbow_flat();
        sv_relic_wrist.setPosition(SV_RELIC_WRIST_DOWN);
    }

    public void relic_arm_ready_delivery() {
        if (!use_relic_elbow) return;
        sv_relic_wrist.setPosition(SV_RELIC_WRIST_UP);
        relic_elbow_up();
        sv_relic_grabber.setPosition(SV_RELIC_GRABBER_CLOSE_NB);
    }

    public void relic_arm_down()
    {
        taintedAccess.stop_chassis();
        taintedAccess.intakeGateInit();
        double pos = sv_relic_wrist.getPosition();
        {
            if (Math.abs(pos - SV_RELIC_WRIST_DOWN_R) > 0.2) {
                while (Math.abs(pos - SV_RELIC_WRIST_DOWN_R) > 0.1) {
                    if (pos < SV_RELIC_WRIST_DOWN_R) {
                        pos += 0.1;
                    } else {
                        pos -= 0.1;
                    }
                    sv_relic_wrist.setPosition(pos);
                    core.sleep(.050);
                    pos = sv_relic_wrist.getPosition();
                }
                sv_relic_wrist.setPosition(SV_RELIC_WRIST_DOWN_R);
            } else {
                sv_relic_wrist.setPosition(SV_RELIC_WRIST_DOWN);
            }
        }
    }

    public void relic_arm_up() {
        taintedAccess.stop_chassis();
        double pos = sv_relic_wrist.getPosition();
        {
            if (Math.abs(pos - SV_RELIC_WRIST_UP) > 0.15) {
                if (pos < SV_RELIC_WRIST_UP) {
                    pos = SV_RELIC_WRIST_UP - 0.2;
                } else {
                    pos = SV_RELIC_WRIST_UP + 0.2;
                }
                sv_relic_wrist.setPosition(pos);
                core.sleep(.300);
                sv_relic_wrist.setPosition(SV_RELIC_WRIST_UP);
            } else {
                sv_relic_wrist.setPosition(SV_RELIC_WRIST_UP);
            }
        }
    }

    public void relic_arm_middle() {
        taintedAccess.stop_chassis();
        sv_relic_wrist.setPosition(SV_RELIC_WRIST_MIDDLE);
    }

}
