package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Put brief class description here...
 */
public class GlyphIntakeSystem {
    public boolean use_verbose = false;
    public boolean use_intake = false;

    final static double SV_INTAKE_GATE_INIT = 0.844;
    final static double SV_INTAKE_GATE_UP = 0.61;
    final static double SV_INTAKE_GATE_MID = 0.5;
    final static double SV_INTAKE_GATE_DOWN = 0.217;
    final static double SV_DUMPER_GATE_INIT = 0.25;
    final static double SV_DUMPER_GATE_UP = 0.25;
    final static double SV_DUMPER_GATE_DOWN = 0.62;

    public DcMotor mt_intake_left = null;
    public DcMotor mt_intake_right = null;
    public Servo sv_intake_gate = null;
    public CRServo sv_bar_wheel = null;

    public double intakeRatio = 1.0;

    // Central core of robot
    CoreSystem core;
    Telemetry ltel;
    ElapsedTime gTime;

    private TaintedAccess taintedAccess;
    void setTaintedAccess(TaintedAccess taintedAccess) {
        this.taintedAccess = taintedAccess;
    }

    GlyphIntakeSystem(CoreSystem core) {
        this.core = core;
    }

    public void enable(boolean isAuto) {
        if (isAuto) {
            use_intake = true;
        } else {
            use_intake = true;
        }
    }

    void disable () {
        use_intake = false;
    }

    void init(HardwareMap hwMap, Telemetry tel, ElapsedTime period) {
        ltel = tel;
        gTime = period;

        sv_intake_gate = hwMap.servo.get("sv_intake_gate");
        sv_intake_gate.setPosition(SV_INTAKE_GATE_INIT);

        if (use_intake) {
            mt_intake_left = hwMap.dcMotor.get("mtIntakeLeft");
            mt_intake_left.setDirection(DcMotor.Direction.REVERSE);
            mt_intake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_intake_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            mt_intake_right = hwMap.dcMotor.get("mtIntakeRight");
            // mt_glyph_slider.setDirection(DcMotor.Direction.REVERSE);
            mt_intake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mt_intake_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (use_verbose) {
            ltel.addData("0: initialize intake CPU time =", "%3.2f sec", period.seconds());
            ltel.update();
        }
    }
    public void correctGlyph(boolean leadClockwise) {
        if (!use_intake)
            return;
        if(leadClockwise) {
            intakeTurn(true);
            core.sleep(150);
            intakeTurn(false);
            core.sleep(150);
            intakeIn();
            core.sleep(300);
        }
        else{
            intakeTurn(false);
            core.sleep(150);
            intakeTurn(true);
            core.sleep(150);
            intakeIn();
            core.sleep(300);
        }
    }
    public void intakeGateInit() {
        if (!use_intake||sv_intake_gate==null)
            return;
        sv_intake_gate.setPosition(SV_INTAKE_GATE_INIT);
    }

    public void intakeGateUp() {
        if (!use_intake || sv_intake_gate==null)
            return;
        double pos = sv_intake_gate.getPosition();
        if (Math.abs(pos-SV_INTAKE_GATE_DOWN)<0.1)
            sv_intake_gate.setPosition(SV_INTAKE_GATE_MID);
        else if (Math.abs(pos-SV_INTAKE_GATE_MID)<0.1)
            sv_intake_gate.setPosition(SV_INTAKE_GATE_UP);
        else
            sv_intake_gate.setPosition(SV_INTAKE_GATE_INIT);
    }

    public void intakeGateDown() {
        if (!use_intake || sv_intake_gate==null)
            return;
        //if (swerve.use_newbot_v2 && robot.relicReachSystem.use_relic_grabber)
        //    robot.relicReachSystem.relic_arm_up();
        double pos = sv_intake_gate.getPosition();
        if (Math.abs(pos-SV_INTAKE_GATE_INIT)<0.1)
            sv_intake_gate.setPosition(SV_INTAKE_GATE_UP);
        else if (Math.abs(pos-SV_INTAKE_GATE_UP)<0.1)
            sv_intake_gate.setPosition(SV_INTAKE_GATE_MID);
        else
            sv_intake_gate.setPosition(SV_INTAKE_GATE_DOWN);
    }

    public void intakeBarWheelIn() {
        if (sv_bar_wheel==null)
            return;
        sv_bar_wheel.setPower(0.8);
    }

    public void intakeBarWheelOut() {
        if (sv_bar_wheel==null)
            return;
        sv_bar_wheel.setPower(-0.8);
    }

    public void intakeBarWheelStop() {
        if (sv_bar_wheel==null)
            return;
        sv_bar_wheel.setPower(0);
    }

    public void intakeIn() {
        if (!use_intake)
            return;
//        if (sv_dumper!=null && robot.sv_dumper.getPosition()<0.63) {
//            return;
//        }
        mt_intake_left.setPower(intakeRatio);
        mt_intake_right.setPower(intakeRatio);
        intakeBarWheelIn();
    }

    public void intakeTurn(boolean clockwise) {
        if (!use_intake)
            return;
        if (clockwise) {
            mt_intake_left.setPower(-intakeRatio / 2.0);
            mt_intake_right.setPower(intakeRatio);
        } else {
            mt_intake_left.setPower(intakeRatio);
            mt_intake_right.setPower(-intakeRatio / 2);
        }
    }

    public void intakeOut() {
        if (!use_intake)
            return;
        mt_intake_left.setPower(-1.0*intakeRatio);
        mt_intake_right.setPower(-1.0*intakeRatio);
        intakeBarWheelOut();
    }

    public void intakeStop() {
        if (!use_intake)
            return;
        mt_intake_left.setPower(0);
        mt_intake_right.setPower(0);
        intakeBarWheelStop();
    }
}
