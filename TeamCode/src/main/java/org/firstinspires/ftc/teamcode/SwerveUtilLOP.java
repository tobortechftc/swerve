package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static java.lang.Thread.sleep;


@TeleOp(name="SwerveDrive: Teleop", group="SwerveDrive")
@Disabled
public class SwerveUtilLOP extends LinearOpMode {

    /* Declare OpMode members. */
      SwerveDriveHardware robot           = new SwerveDriveHardware();

    /**
     * Is used for checking or determining a color based on an alliance
     * Can be used to set opposing or team alliance color
     */
    enum TeamColor {
        RED, BLUE, UNKNOWN;

        private TeamColor opposite;

        static {
            RED.opposite = BLUE;
            BLUE.opposite = RED;
            UNKNOWN.opposite = UNKNOWN;
        }

        public TeamColor getOpposingColor(){
            return opposite;
        }
    }
    enum RangeSensor{
        FRONT_LEFT, FRONT_RIGHT, BACK
    }

    @Override
    public void runOpMode() throws InterruptedException{
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello! Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }

    public void init_and_test() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        if (robot.use_glyph_grabber) {
            // test_glyph_rotator_encoder();
            robot.gg_rotator_encoder_ok = true;
            // test_glyph_slider_encoder();
            robot.gg_slider_encoder_ok = true;
        }
        if (robot.use_dumper) {
            robot.gg_slider_encoder_ok = true;
        }
    }

    public void start_init() throws InterruptedException {
        robot.runtimeAuto.reset();
        if (robot.use_glyph_grabber) {
            glyph_grabber_auto_open();
            glyph_slider_init();
        }
        //if (robot.use_dumper) {
        //    lift_to_target(robot.LIFT_INIT_COUNT);
        //}
        if (robot.use_newbot_v2 && robot.use_arm) {
            robot.sv_jkicker.setPosition(robot.SV_JKICKER_UP);
        }
        if (robot.use_relic_grabber) {
            if (robot.use_newbot) {
                relic_grabber_open(false);
            } else {
                relic_grabber_open(false);
            }
            relic_arm_auto();
        }
        if (robot.use_intake) {
            intakeGateUp();
        }
        if (robot.use_Vuforia) {
            robot.targetColumn = get_cryptobox_column();
            telemetry.addData("0: Crypto Column =", robot.targetColumn);
            telemetry.update();
        }
        if (robot.use_imu) {
            if (robot.imu.getSystemStatus()== BNO055IMU.SystemStatus.SYSTEM_ERROR && robot.imu2!=null) {
                robot.use_imu2 = true;
            }
        }
        if (robot.use_verbose)
            telemetry.addData("0: End start_init CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
    }

    public double imu_heading() {
        if (!robot.use_imu)
            return 999;

        if(!robot.use_newbot) {
            robot.angles = (robot.use_imu2 ? robot.imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES) :
                    robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            return robot.angles.firstAngle;
        }
        else{
            if(!robot.use_imu2){
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                return robot.angles.firstAngle;
            }
            else{
                robot.angles = robot.imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                return robot.angles.firstAngle;
            }
        }
    }
    public double imu2_heading() {
        if (!robot.use_imu || (robot.imu2==null))
            return 999;

        robot.angles = robot.imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return robot.angles.firstAngle;

    }

    public void dumper_vertical() {
        if (!robot.use_dumper)
            return;
        robot.sv_dumper.setPosition(robot.SV_DUMPER_UP);
        if (robot.use_dumper_gate) {
            dumperGateDown();
        }
    }

    public void dumper_higher() {
        if (!robot.use_dumper)
            return;
        double pos = robot.sv_dumper.getPosition();
        if (pos>robot.SV_DUMPER_DUMP)
            pos -= 0.02;

        robot.sv_dumper.setPosition(pos);
    }

    public void dumper_lower() {
        if (!robot.use_dumper)
            return;
        double pos = robot.sv_dumper.getPosition();
        if (pos<robot.SV_DUMPER_DOWN)
            pos += 0.03;

        robot.sv_dumper.setPosition(pos);
    }

    public void dumper_up() {
        if (!robot.use_dumper)
            return;
        double pos = robot.sv_dumper.getPosition();
        if (Math.abs(pos-robot.SV_DUMPER_DOWN)<0.05)
            robot.sv_dumper.setPosition(robot.SV_DUMPER_HALF_UP);
        else if (Math.abs(pos-robot.SV_DUMPER_HALF_UP)<0.05) {
            robot.sv_dumper.setPosition(robot.SV_DUMPER_UP);
            if (robot.use_dumper_gate) {
                dumperGateDown();
            }
        } else {
            robot.sv_dumper.setPosition(robot.SV_DUMPER_DUMP);
            if (robot.use_dumper_gate) {
                dumperGateDown();
            }
        }
    }

    public void dumper_shake() {
        if (!robot.use_dumper)
            return;
        double pos = robot.sv_dumper.getPosition();
        if ((pos-0.1)>0) {
            robot.sv_dumper.setPosition(pos - 0.1);
            sleep(200);
        }
        robot.sv_dumper.setPosition(pos);
    }

    public void dumper_down(boolean raiseGate)
    {
        if (!robot.use_dumper)
            return;
        robot.sv_dumper.setPosition(robot.SV_DUMPER_DOWN);
        if (robot.use_dumper_gate && raiseGate) {
            dumperGateUp();
        }
    }

    public void glyph_grabber_bottom_closer() {
        if (robot.is_gg_upside_down) { // close up grabber a little bit
            double cur_pos = robot.sv_glyph_grabber_top.getPosition();
            if (cur_pos<0.9)
                cur_pos += 0.01;
            robot.sv_glyph_grabber_top.setPosition(cur_pos);
        } else {
            double cur_pos = robot.sv_glyph_grabber_bottom.getPosition();
            if (cur_pos>0.1)
                cur_pos -= 0.01;
            robot.sv_glyph_grabber_bottom.setPosition(cur_pos);
        }
    }
    public void glyph_grabber_bottom_widen() {
        if (robot.is_gg_upside_down) { // widen up grabber a little bit
            double cur_pos = robot.sv_glyph_grabber_top.getPosition();
            if (cur_pos>0.1)
                cur_pos -= 0.01;
            robot.sv_glyph_grabber_top.setPosition(cur_pos);
        } else {
            double cur_pos = robot.sv_glyph_grabber_bottom.getPosition();
            if (cur_pos<0.9)
                cur_pos += 0.01;
            robot.sv_glyph_grabber_bottom.setPosition(cur_pos);
        }
    }

    public void glyph_grabber_top_closer() {
        if (robot.is_gg_upside_down) { // close up grabber a little bit
            double cur_pos = robot.sv_glyph_grabber_bottom.getPosition();
            if (cur_pos>0.1)
                cur_pos -= 0.01;
            robot.sv_glyph_grabber_bottom.setPosition(cur_pos);
        } else {
            double cur_pos = robot.sv_glyph_grabber_top.getPosition();
            if (cur_pos<0.9)
                cur_pos += 0.01;
            robot.sv_glyph_grabber_top.setPosition(cur_pos);
        }
    }

    public void glyph_grabber_top_widen() {
        if (robot.is_gg_upside_down) { // widen up grabber a little bit
            double cur_pos = robot.sv_glyph_grabber_bottom.getPosition();
            if (cur_pos<0.9)
                cur_pos += 0.05;
            robot.sv_glyph_grabber_bottom.setPosition(cur_pos);
        } else {
            double cur_pos = robot.sv_glyph_grabber_top.getPosition();
            if (cur_pos>0.1)
                cur_pos -= 0.05;
            robot.sv_glyph_grabber_top.setPosition(cur_pos);
        }
    }

    public void glyph_grabber_close() {
        stop_chassis();
        if (robot.is_gg_upside_down) { // close up grabber
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_CLOSED);
        } else {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_CLOSED);
        }
    }

    public void glyph_grabber_auto_close(boolean is_top, boolean auto) { // close the down/up grabber depend on upside down
        stop_chassis();
        boolean close_top = (robot.is_gg_upside_down && !is_top) ||
                            (!robot.is_gg_upside_down && is_top);
        if (close_top) { // close up grabber
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_CLOSED);
            sleep(200);
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_HALF_CLOSED);
            sleep(200);
            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
            if (auto) {
                driveTT(-0.6, -0.6);
                sleep(500);
            } else {
                driveTT(-0.4, -0.4);
                sleep(100);
            }
            driveTT(0,0);
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_CLOSED);
            robot.gg_top_close = true;
        } else {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_CLOSED);
            sleep(200);
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_HALF_CLOSED);
            sleep(200);
            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
            if (auto) {
                driveTT(-0.6, -0.6);
                sleep(500);
            } else {
                driveTT(-0.4, -0.4);
                sleep(100);
            }
            driveTT(0,0);
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_CLOSED);
            robot.gg_bottom_close = true;

        }
    }

    public void glyph_grabber_all_close() { // close the down/up grabber depend on upside down
        stop_chassis();
        robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_CLOSED);
        robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_CLOSED);
        robot.gg_top_close = true;
        robot.gg_bottom_close = true;

    }

    public void glyph_grabber_half_close() {
        stop_chassis();
        if (robot.is_gg_upside_down) { // close up grabber
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_HALF_CLOSED);
            robot.gg_top_close = true;
        } else {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_HALF_CLOSED);
            robot.gg_bottom_close = true;
        }
    }

    public void glyph_grabber_half_close_both() {
        stop_chassis();
        robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_HALF_CLOSED);
        robot.gg_top_close = true;
        robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_HALF_CLOSED);
        robot.gg_bottom_close = true;
    }

    public void glyph_grabber_auto_open() { // open both grabbers
        stop_chassis();
        robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_OPEN);
        robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_OPEN);
        robot.gg_top_close = false;
        robot.gg_bottom_close = false;
        // auto rotate back to top up
        // sleep(500);
        //if (robot.is_gg_upside_down)
        //    glyph_grabber_auto_rotate(0.3);
    }

    public void glyph_grabber_open_top() { // open top grabber
        stop_chassis();
        if (robot.is_gg_upside_down) {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_OPEN);
        } else {
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_OPEN);
        }
    }

    public void glyph_grabber_open_bottom() { // open top grabber
        stop_chassis();
        if (robot.is_gg_upside_down) {
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_OPEN);
        } else {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_OPEN);
        }
    }

    public void glyph_grabber_open_and_push() { // open both grabbers
        stop_chassis();
        glyph_grabber_auto_open();
        sleep(500);
        driveTT(-0.2,-0.2);
        sleep(500);
        driveTT(0.6,0.6);
        sleep(100);
        driveTT(0,0);
    }

    public void glyph_grabber_open_and_push_auto() { // open both grabbers
        stop_chassis();
        glyph_grabber_auto_open();
        driveTT(-0.2,-0.2);
        sleep(500);
        driveTT(0.6,0.6);
        sleep(100);
        driveTT(0,0);
    }

    public void glyph_grabber_auto_init() {
        glyph_slider_back_init();
    }

    public void glyph_grabber_auto_rotate(double power) {
        stop_chassis();
        // if grabber is close and at ladder 0, need to slide up a little bit before rotation
        boolean need_slide_up = false;
        stop_chassis();

        int orig_slide_pos = robot.mt_glyph_rotator.getCurrentPosition();
        int orig_slide_pos2 = robot.mt_glyph_rotator.getCurrentPosition();
        int count = 0;
        while (Math.abs(orig_slide_pos-orig_slide_pos2)>10 && count<5) {
            orig_slide_pos = robot.mt_glyph_rotator.getCurrentPosition();
            orig_slide_pos2 = robot.mt_glyph_rotator.getCurrentPosition();
            count ++;
        }
        if (count==5) {
            return; // encoder error, abort the mission
        }
        if (robot.gg_layer==0 && orig_slide_pos<600) {
            need_slide_up = true;
        }
        if (orig_slide_pos>1000) { // more than 4.5 inches above the ground
            need_slide_up = false;
        }
        need_slide_up = false;
        double up_inches = (1000-orig_slide_pos) / 300.0;
        if (need_slide_up && (up_inches>0)) {
            glyph_slider_up_inches(robot.GG_SLIDE_UP_POWER, up_inches);
            sleep(300);
        }
        // rotate 180 degrees back and forth
        int cur_count = robot.orig_rot_pos; // robot.mt_test.getCurrentPosition();
        if (robot.is_gg_upside_down) { // back to orig pos
            robot.target_rot_pos = robot.orig_rot_pos;
        } else {
            // robot.orig_rot_pos = cur_count;
            robot.target_rot_pos = cur_count + (int) (180.0 / 360.0 * robot.ONE_ROTATION_60);
        }
        rotate_to_target(power);
        int new_slide_pos = robot.mt_glyph_rotator.getCurrentPosition();
        if (Math.abs(new_slide_pos-orig_slide_pos)<100) { // don't rotate correctly
            return;
        }
        robot.is_gg_upside_down = !robot.is_gg_upside_down;
        // boolean need_slide_back = need_slide_up;
        boolean need_slide_back = false;
        if (robot.is_gg_upside_down && robot.gg_top_close)
            need_slide_back = false;
        if (!robot.is_gg_upside_down && robot.gg_bottom_close)
            need_slide_back = false;
        if (need_slide_back) {
            int back_pos = orig_slide_pos;
            if (back_pos<robot.init_gg_slider_pos+50) {
                back_pos += 100;
                if (robot.is_gg_upside_down)
                    back_pos += 150;
            }
            if (back_pos>orig_slide_pos) {
                glyph_slider_position(back_pos);
                sleep(500);
            }
        }
    }

    public void rotate_refine() {
        rotate_to_target(0.2);
    }

    public void rotate_refine_up() {
        stop_chassis();
        robot.target_rot_pos = robot.target_rot_pos + 20;
        rotate_to_target(0.2);
    }

    public void rotate_refine_down() {
        stop_chassis();
        robot.target_rot_pos = robot.target_rot_pos - 20;
        rotate_to_target(0.2);
    }

    public void relic_grabber_lower() {
        double pos = robot.sv_relic_grabber.getPosition()-0.04;
        if (pos<0.001) pos = 0.001;
        robot.sv_relic_grabber.setPosition(pos);
    }

    public void relic_grabber_higher() {
        double pos = robot.sv_relic_grabber.getPosition()+0.04;
        if (pos>0.99) pos = 0.99;
        robot.sv_relic_grabber.setPosition(pos);
    }

    public void relic_grabber_close() {
        if (robot.use_newbot) {
            robot.sv_relic_grabber.setPosition(robot.SV_RELIC_GRABBER_CLOSE_NB);
        } else {
            robot.sv_relic_grabber.setPosition(robot.SV_RELIC_GRABBER_CLOSE);
        }
    }

    public void relic_grabber_open(boolean wide) {
        if (robot.use_newbot) {
            if (wide) {
                robot.sv_relic_grabber.setPosition(robot.SV_RELIC_GRABBER_OPEN_W_NB);
            } else {
                robot.sv_relic_grabber.setPosition(robot.SV_RELIC_GRABBER_OPEN_NB);
            }
        } else {
            robot.sv_relic_grabber.setPosition(robot.SV_RELIC_GRABBER_OPEN);
        }
    }

    public void relic_grabber_release() {
        // robot.sv_relic_wrist.setPosition(robot.SV_RELIC_ARM_DOWN_R);
        double pos = robot.sv_relic_grabber.getPosition();
        double tar = (robot.use_newbot?robot.SV_RELIC_GRABBER_OPEN_NB:robot.SV_RELIC_GRABBER_OPEN);
        if (pos < tar) {
            robot.sv_relic_grabber.setPosition(pos+0.1);
            sleep(250);
            robot.sv_relic_grabber.setPosition(pos+0.2);
            sleep(250);
        } else {
            robot.sv_relic_grabber.setPosition(pos-0.1);
            sleep(250);
            robot.sv_relic_grabber.setPosition(pos-0.2);
            sleep(250);
        }
        robot.sv_relic_grabber.setPosition(tar);
    }

    public void auto_relic_release() {
        stop_chassis();
        double cur_pos = robot.sv_relic_wrist.getPosition();
        if (Math.abs(cur_pos - robot.SV_RELIC_WRIST_DOWN) > 0.1) {
            relic_arm_down();
            sleep(500);
        }
        if (Math.abs(cur_pos - robot.SV_RELIC_WRIST_DOWN) > 0.02) {
            relic_arm_down();
            sleep(500);
        }
        relic_grabber_release();
        sleep(250);
        if (robot.use_newbot) {
            relic_slider_in(0.5, true);
            sleep(400);
        } else {
            relic_slider_in(1.0, true);
            sleep(1000);
        }
        relic_slider_stop();
        relic_arm_up();

    }

    public void relic_slider_in(double power, boolean force) {
        if (Math.abs(power)>1) power=1;
        double pos = robot.mt_relic_slider.getCurrentPosition();
        if (pos>-100 && power!=0 && force==false) {
            if (pos<0) power = 0.1;
            else {
                power = 0;
                robot.mt_relic_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.mt_relic_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        if (power>0) stop_chassis();
        robot.mt_relic_slider.setPower(Math.abs(power));
    }

    public void relic_slider_out(double power) {
        if (Math.abs(power)>1) power=1;
        if (power>0) stop_chassis();
        robot.mt_relic_slider.setPower(-1*Math.abs(power));
    }

    public void relic_slider_stop() {
        robot.mt_relic_slider.setPower(0);
    }

    public void relic_elbow_up_auto() {
        // down -> flat -> up -> init
        if (!robot.use_relic_elbow) return;
        if (robot.relic_arm_pos == SwerveDriveHardware.RelicArmPos.DOWN) {
            relic_elbow_flat();
            relic_arm_ready_grab_and_release();
        } else if (robot.relic_arm_pos == SwerveDriveHardware.RelicArmPos.FLAT) {
            // relic_elbow_up();
            relic_arm_ready_delivery();
        } else {
            relic_elbow_init();
        }
    }

    public void relic_elbow_down_auto() {
        // init -> up -> flat -> down
        if (!robot.use_relic_elbow) return;
        if (robot.relic_arm_pos == SwerveDriveHardware.RelicArmPos.INIT) {
            relic_elbow_up();
        } else if (robot.relic_arm_pos == SwerveDriveHardware.RelicArmPos.UP) {
            // relic_elbow_flat();
            relic_arm_ready_grab_and_release();
        } else {
            relic_elbow_down();
        }
    }

    public void relic_elbow_higher() {
        if (!robot.use_relic_elbow) return;
        double new_pos = robot.sv_relic_elbow.getPosition() + 0.001;
        if (new_pos>0.999) new_pos = 0.999;
        robot.sv_relic_elbow.setPosition(new_pos);
        if (new_pos>=robot.SV_RELIC_ELBOW_INIT)
            robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.INIT;
        else if (new_pos>=robot.SV_RELIC_ELBOW_UP)
            robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.UP;
        else if (new_pos>=robot.SV_RELIC_ELBOW_FLAT)
            robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.FLAT;
        else
            robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.DOWN;
    }

    public void relic_elbow_lower() {
        if (!robot.use_relic_elbow) return;
        double new_pos = robot.sv_relic_elbow.getPosition() - 0.001;
        if (new_pos<0.001) new_pos = 0.001;
        robot.sv_relic_elbow.setPosition(new_pos);
        if (new_pos<=robot.SV_RELIC_ELBOW_DOWN)
            robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.DOWN;
        else if (new_pos<=robot.SV_RELIC_ELBOW_FLAT)
            robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.FLAT;
        else if (new_pos<=robot.SV_RELIC_ELBOW_UP)
            robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.UP;
        else
            robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.INIT;
    }

    public void relic_elbow_up() {
        if (!robot.use_relic_elbow) return;
        robot.sv_relic_elbow.setPosition(robot.SV_RELIC_ELBOW_UP);
        robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.UP;
    }

    public void relic_elbow_flat() {
        if (!robot.use_relic_elbow) return;
        robot.sv_relic_elbow.setPosition(robot.SV_RELIC_ELBOW_FLAT);
        robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.FLAT;
    }

    public void relic_elbow_down() {
        if (!robot.use_relic_elbow) return;
        robot.sv_relic_elbow.setPosition(robot.SV_RELIC_ELBOW_DOWN);
        robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.DOWN;
    }

    public void relic_elbow_init() {
        if (!robot.use_relic_elbow) return;
        robot.sv_relic_elbow.setPosition(robot.SV_RELIC_ELBOW_INIT);
        robot.relic_arm_pos = SwerveDriveHardware.RelicArmPos.INIT;
    }

    public void relic_arm_ready_grab_and_release() {
        if (!robot.use_relic_elbow) return;
        relic_elbow_flat();
        robot.sv_relic_wrist.setPosition(robot.SV_RELIC_WRIST_DOWN);
    }

    public void relic_arm_ready_delivery() {
        if (!robot.use_relic_elbow) return;
        robot.sv_relic_wrist.setPosition(robot.SV_RELIC_WRIST_UP);
        relic_elbow_up();
        robot.sv_relic_grabber.setPosition(robot.SV_RELIC_GRABBER_CLOSE_NB);
    }

    public void relic_arm_auto() {
        stop_chassis();
        if (robot.use_newbot) {
            // robot.sv_relic_wrist.setPosition(robot.SV_RELIC_WRIST_DOWN_AUTO);
            relic_arm_down();
        } else {
            relic_arm_down();
        }
    }

    public void relic_arm_down()
    {
        stop_chassis();
        if (robot.use_intake) {
            intakeGateInit();
        }
        double pos = robot.sv_relic_wrist.getPosition();
        if (robot.use_newbot) {
            if (Math.abs(pos - robot.SV_RELIC_WRIST_DOWN_R) > 0.2) {
                while (Math.abs(pos - robot.SV_RELIC_WRIST_DOWN_R) > 0.1) {
                    if (pos < robot.SV_RELIC_WRIST_DOWN_R) {
                        pos += 0.1;
                    } else {
                        pos -= 0.1;
                    }
                    robot.sv_relic_wrist.setPosition(pos);
                    sleep(150);
                    pos = robot.sv_relic_wrist.getPosition();
                }
                robot.sv_relic_wrist.setPosition(robot.SV_RELIC_WRIST_DOWN_R);
            } else {
                robot.sv_relic_wrist.setPosition(robot.SV_RELIC_WRIST_DOWN);
            }
        } else {
            if (Math.abs(pos - robot.SV_RELIC_ARM_DOWN_R) > 0.2) {
                while (Math.abs(pos - robot.SV_RELIC_ARM_DOWN_R) > 0.1) {
                    if (pos < robot.SV_RELIC_ARM_DOWN_R) {
                        pos = robot.SV_RELIC_ARM_DOWN_R - 0.1;
                    } else {
                        pos = robot.SV_RELIC_ARM_DOWN_R + 0.1;
                    }
                    robot.sv_relic_wrist.setPosition(pos);
                    sleep(350);
                }
                robot.sv_relic_wrist.setPosition(robot.SV_RELIC_ARM_DOWN_R);
            } else {
                robot.sv_relic_wrist.setPosition(robot.SV_RELIC_ARM_DOWN);
            }
        }
    }

    public void relic_arm_up() {
        stop_chassis();
        double pos = robot.sv_relic_wrist.getPosition();
        if (robot.use_newbot) {
            if (Math.abs(pos - robot.SV_RELIC_WRIST_UP) > 0.15) {
                if (pos < robot.SV_RELIC_WRIST_UP) {
                    pos = robot.SV_RELIC_WRIST_UP - 0.2;
                } else {
                    pos = robot.SV_RELIC_WRIST_UP + 0.2;
                }
                robot.sv_relic_wrist.setPosition(pos);
                sleep(300);
                robot.sv_relic_wrist.setPosition(robot.SV_RELIC_WRIST_UP);
            } else {
                robot.sv_relic_wrist.setPosition(robot.SV_RELIC_WRIST_UP);
            }
        } else {
            if (Math.abs(pos - robot.SV_RELIC_ARM_UP) > 0.15) {
                if (pos < robot.SV_RELIC_ARM_UP) {
                    pos = robot.SV_RELIC_ARM_UP - 0.2;
                } else {
                    pos = robot.SV_RELIC_ARM_UP + 0.2;
                }
                robot.sv_relic_wrist.setPosition(pos);
                sleep(300);
                robot.sv_relic_wrist.setPosition(robot.SV_RELIC_ARM_UP);
            } else {
                robot.sv_relic_wrist.setPosition(robot.SV_RELIC_ARM_UP + 0.12);
            }
        }
    }

    public void relic_arm_middle() {
        stop_chassis();
        if (robot.use_newbot) {
            robot.sv_relic_wrist.setPosition(robot.SV_RELIC_WRIST_MIDDLE);
        } else {
            robot.sv_relic_wrist.setPosition(robot.SV_RELIC_ARM_MIDDLE);
        }
    }

    public void relic_slider_out_max() {
        robot.target_relic_slider_pos = robot.RELIC_SLIDE_MAX;
        relic_slide_to_target(1.0);
    }

    public void relic_slider_back_auto() {
        robot.target_relic_slider_pos = robot.RELIC_SLIDE_MAX/2;
        relic_slide_to_target(1.0);
        relic_arm_up();
        robot.target_relic_slider_pos = 0;
        relic_slide_to_target(1.0);
    }

    public void test_rotate(double power) {
        // test rotation 180 degrees back and forth
        int cur_count = robot.orig_rot_pos; // robot.mt_test.getCurrentPosition();
        if (robot.is_gg_upside_down) { // back to orig pos
            robot.target_rot_pos = robot.orig_rot_pos;
        } else {
            // robot.orig_rot_pos = cur_count;
            robot.target_rot_pos = cur_count + (int) (180.0 / 360.0 * robot.ONE_ROTATION_60);
        }
        //test_rotate_to_target(power);
        robot.is_gg_upside_down = !robot.is_gg_upside_down;
    }

    public void rotate_to_target(double power) {
        if (!robot.gg_rotator_encoder_ok)
            return;
        stop_chassis(); // ensure chassis stops
        robot.mt_glyph_rotator.setTargetPosition(robot.target_rot_pos);
        robot.mt_glyph_rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.runtime.reset();
        robot.mt_glyph_rotator.setPower(Math.abs(power));
        while (robot.mt_glyph_rotator.isBusy() && (robot.runtime.seconds()<3) && opModeIsActive()) {
            telemetry.addData("8. gg-rot pwr/cur/tar = ","%3.2f/%d/%d(%s)",
                    robot.mt_glyph_rotator.getPower(),robot.mt_glyph_rotator.getCurrentPosition(),robot.target_rot_pos,
                    (robot.is_gg_upside_down ?"dw":"up"));
            telemetry.update();
        }
        robot.mt_glyph_rotator.setPower(Math.abs(power/2.0));
        while (robot.mt_glyph_rotator.isBusy() && (robot.runtime.seconds()<1) && opModeIsActive()) {
            telemetry.addData("8. gg-rot pwr/cur/tar = ","%3.2f/%d/%d(%s)",
                    robot.mt_glyph_rotator.getPower(),robot.mt_glyph_rotator.getCurrentPosition(),robot.target_rot_pos,
                    (robot.is_gg_upside_down ?"dw":"up"));
            telemetry.update();
        }
        robot.mt_glyph_rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mt_glyph_rotator.setPower(0);
    }

    public void glyph_grabber_rotate(double power, double degree) {
        if (!robot.gg_rotator_encoder_ok)
            return;

        double adjust_r = 1.0;
        if (power>0) {
            adjust_r = 1.0;
        } else {
            adjust_r = 1.05;
        }

        // robot.mt_glyph_rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.mt_glyph_rotator.setPower(power);
        int cur_count = robot.mt_glyph_rotator.getCurrentPosition();
        double target_count = degree/360.0 * robot.ONE_ROTATION_60 * adjust_r;
        if (power>0) {
            target_count = cur_count + target_count;
        } else {
            target_count = cur_count - target_count;
        }
        robot.mt_glyph_rotator.setTargetPosition((int)target_count);
        robot.mt_glyph_rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.runtime.reset();
        robot.mt_glyph_rotator.setPower(Math.abs(power));
        while (robot.mt_glyph_rotator.isBusy() && (robot.runtime.seconds()<3) && opModeIsActive()) {
            telemetry.addData("8. gg-rot pwr/cnt/down = ","%3.2f/%d/%s",
                    robot.mt_glyph_rotator.getPower(),robot.mt_glyph_rotator.getCurrentPosition(),
                    (robot.is_gg_upside_down ?"down":"up"));
            telemetry.update();
        }
        robot.mt_glyph_rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mt_glyph_rotator.setPower(0);
    }

    void glyph_slider_init() {
        robot.target_gg_slider_pos = (int)(robot.GG_SLIDE_INIT);
        slide_to_target(.8);
        robot.gg_layer = 0;
    }

    void glyph_slider_position(int pos) {
        robot.target_gg_slider_pos = pos;
        slide_to_target(.5);
    }

    void glyph_slider_up_inches(double power, double in) {
        int count = (int)(in / robot.GG_SLIDE_INCHES_PER_ROTATION * robot.ONE_ROTATION_60);
        robot.target_gg_slider_pos = robot.init_gg_slider_pos + count;
        slide_to_target(power);
    }

    void glyph_slider_up_auto() {
        if (robot.gg_layer<robot.max_gg_layer)
            robot.gg_layer ++;
        robot.target_gg_slider_pos = robot.layer_positions[robot.gg_layer];
        slide_to_target(robot.GG_SLIDE_UP_POWER);
    }

    void glyph_slider_down_auto() {
        if (robot.gg_layer>0)
            robot.gg_layer --;
        robot.target_gg_slider_pos = robot.layer_positions[robot.gg_layer];
        slide_to_target(robot.GG_SLIDE_DOWN_POWER);
    }

    void glyph_slider_back_init() { // back to initial position
        robot.target_gg_slider_pos = robot.init_gg_slider_pos;
        slide_to_target(robot.GG_SLIDE_DOWN_POWER);
        robot.gg_layer = 0;
    }

    void lift_back_init() { // back to initial position
        robot.target_gg_slider_pos = robot.init_gg_slider_pos;
        slide_to_target(robot.GG_SLIDE_DOWN_POWER);
        dumper_down(true);
    }

    void lift_up_level_half() {
        robot.target_gg_slider_pos = robot.layer_positions[1] / 2;
        lift_up(false);
        slide_to_target(robot.GG_SLIDE_UP_POWER);
    }

    void lift_up(boolean force) {
        double power = 1.0;
        // never exceed GG_SLIDE_MAX_COUNT
        int cur_pos = robot.mt_lift.getCurrentPosition();
        if ((cur_pos>robot.LIFT_MAX_COUNT) && !force) {
            power = 0.0;
        }
        if (robot.sv_dumper.getPosition()>robot.SV_DUMPER_LIFT) {
            robot.sv_dumper.setPosition(robot.SV_DUMPER_LIFT);
        }
        robot.mt_lift.setPower(power);
    }

    void lift_down(boolean force) {
        double power = -0.95;
        // never lower than 0
        int cur_pos = robot.mt_lift.getCurrentPosition();
        if ((cur_pos<=robot.LIFT_INIT_COUNT) && !force) {
            power = 0.0;
            dumper_down(true);
        }
        robot.mt_lift.setPower(power);
    }

    void lift_up_and_down(boolean force) {
        lift_up(force);
        sleep(300);
        lift_stop();
        sleep(150);
        lift_down(force);
        sleep(150);
        lift_stop();
    }

    void lift_stop() {
        robot.mt_lift.setPower(0.0);
    }

    void glyph_slider_up() {
        robot.mt_glyph_slider_pw = 1.0;
        // never exceed GG_SLIDE_MAX_COUNT
        int cur_pos = robot.mt_glyph_slider.getCurrentPosition();
        if (cur_pos>robot.GG_SLIDE_MAX_COUNT)
            robot.mt_glyph_slider_pw = 0.0;
        for (int i=robot.max_gg_layer; i>=0; i--) {
            if (cur_pos < robot.layer_positions[i])
                break;
            robot.gg_layer = i;
        }
        robot.mt_glyph_slider.setPower(robot.mt_glyph_slider_pw);
    }

    void glyph_slider_down() {
        robot.mt_glyph_slider_pw = -0.9;
        // never lower than 0
        int cur_pos = robot.mt_glyph_slider.getCurrentPosition();
        if (cur_pos<=0)
            robot.mt_glyph_slider_pw = 0.0;
        for (int i=0; i<robot.max_gg_layer; i++) {
            if (cur_pos < robot.layer_positions[i])
                break;
            robot.gg_layer = i;
        }
        robot.mt_glyph_slider.setPower(robot.mt_glyph_slider_pw);
    }

    void glyph_slider_up_force() { // force to go negative and reset encode
        robot.mt_glyph_slider_pw = 0.7;
        int cur_pos = robot.mt_glyph_slider.getCurrentPosition();
        for (int i=robot.max_gg_layer; i>=0; i--) {
            if (cur_pos < robot.layer_positions[i])
                break;
            robot.gg_layer = i;
        }
        robot.mt_glyph_slider.setPower(robot.mt_glyph_slider_pw);
    }

    void glyph_slider_down_and_reset() { // force to go negative and reset encode
        robot.mt_glyph_slider_pw = -0.4;
        robot.mt_glyph_slider.setPower(robot.mt_glyph_slider_pw);
        int cur_pos = robot.mt_glyph_slider.getCurrentPosition();
        if (cur_pos<0) {
            robot.gg_layer = 0;
            robot.mt_glyph_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mt_glyph_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void glyph_slider_stop() {
        robot.mt_glyph_slider_pw = 0.0;
        robot.mt_glyph_slider.setPower(robot.mt_glyph_slider_pw);
    }

    void test_glyph_rotator_encoder() {
        robot.mt_glyph_rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mt_glyph_rotator.setPower(.6);
        sleep(20);
        robot.mt_glyph_rotator.setPower(0);
        if (robot.mt_glyph_rotator.getCurrentPosition()!=0)
            robot.gg_rotator_encoder_ok = true;
        else
            robot.gg_rotator_encoder_ok = false;
        robot.mt_glyph_rotator.setPower(-0.6);
        sleep(20);
        robot.mt_glyph_rotator.setPower(0);
    }

    void test_glyph_slider_encoder() {
        robot.mt_glyph_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mt_glyph_slider.setPower(.6);
        sleep(20);
        robot.mt_glyph_slider.setPower(0);
        if (robot.mt_glyph_slider.getCurrentPosition()!=0)
            robot.gg_slider_encoder_ok = true;
        else
            robot.gg_slider_encoder_ok = false;
    }

    public void relic_slide_to_target(double power) {
        if (!robot.gg_slider_encoder_ok)
            return;

        stop_chassis(); // ensure chassis stops

        if (power<0) power=-1.0*power;
        robot.mt_relic_slider.setTargetPosition(robot.target_relic_slider_pos);
        robot.mt_relic_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.runtime.reset();
        robot.mt_relic_slider.setPower(Math.abs(power));
        while (robot.mt_relic_slider.isBusy() && (robot.runtime.seconds()<5) && opModeIsActive()) {
            telemetry.addData("9.2 relic pwr/cur/tar = ","%3.2f/%d/%d",
                    robot.mt_relic_slider.getPower(),robot.mt_relic_slider.getCurrentPosition(),robot.target_relic_slider_pos);
            telemetry.update();
        }

        robot.mt_relic_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mt_relic_slider.setPower(0);
    }

    public void lift_to_target(int pos) {
        if (!robot.gg_slider_encoder_ok)
            return;
        robot.target_gg_slider_pos = pos;
        slide_to_target(0.5);
    }

    public void slide_to_target(double power) {
        if (!robot.gg_slider_encoder_ok)
            return;

        stop_chassis(); // ensure chassis stops

        if (power<0) power=-1.0*power;
        DcMotor mt = (robot.use_newbot? robot.mt_lift:robot.mt_glyph_slider);

        mt.setTargetPosition(robot.target_gg_slider_pos);
        mt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.runtime.reset();
        mt.setPower(Math.abs(power));
        while (mt.isBusy() && (robot.runtime.seconds() < 3) && opModeIsActive()) {
            telemetry.addData("8. gg-rot pwr/cur/tar = ", "%3.2f/%d/%d",
                    mt.getPower(), mt.getCurrentPosition(), robot.target_gg_slider_pos);
            telemetry.update();
        }
        mt.setPower(Math.abs(power / 2.0));
        while (mt.isBusy() && (robot.runtime.seconds() < 1) && opModeIsActive()) {
            telemetry.addData("8. gg-rot pwr/cur/tar = ", "%3.2f/%d/%d",
                    mt.getPower(), mt.getCurrentPosition(), robot.target_gg_slider_pos);
            telemetry.update();
        }
        mt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mt.setPower(0);
    }

    public void driveTT(double lp, double rp) {
        if(!robot.fast_mode && robot.straight_mode) { // expect to go straight
            if (robot.use_imu) {
                double cur_heading = imu_heading();
                double heading_off_by = ((cur_heading - robot.target_heading) / 360);
                if(robot.use_swerve|| robot.use_newbot) {
                    if(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                        if(rp > 0 && lp > 0) { //When going forward
                            if(robot.use_swerve) {
                                if (cur_heading - robot.target_heading > 0.7) {
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION - heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION - heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) {
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION + heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION + heading_off_by);
                                }
                            } else if (robot.use_newbot) {
                                if (cur_heading - robot.target_heading > 0.7) {
                                    robot.servoFrontLeft.setPosition(robot.NB_SERVO_FL_FORWARD_POSITION - heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) {
                                    robot.servoFrontLeft.setPosition(robot.NB_SERVO_FL_FORWARD_POSITION + heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                                }
                            }

                        }
                        else{ // When going backward
                            if(robot.use_swerve) {
                                if (cur_heading - robot.target_heading > 0.7) { //Drifting to the left
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION + heading_off_by); //Turn Front servos to the right
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION + heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) { //Drifting to the right
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION - heading_off_by); //Turn Front servos to the left
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION - heading_off_by);
                                }
                            } if (robot.use_newbot) {
                                if (cur_heading - robot.target_heading > 0.7) { //Drifting to the left
                                    robot.servoFrontLeft.setPosition(robot.NB_SERVO_FL_FORWARD_POSITION + heading_off_by); //Turn Front servos to the right
                                    robot.servoFrontRight.setPosition(robot.NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) { //Drifting to the right
                                    robot.servoFrontLeft.setPosition(robot.NB_SERVO_FL_FORWARD_POSITION - heading_off_by); //Turn Front servos to the left
                                    robot.servoFrontRight.setPosition(robot.NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                                }
                            }
                        }
                    }
                    else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB){  //Tentative, could stand to remove
                        if (cur_heading - robot.target_heading > 0.7) { // crook to left,  slow down right motor
                            if (rp > 0) rp *= 0.7; //If the robot is going forward
                            else lp *= 0.7; // If the robot is going backwards
                        } else if (cur_heading - robot.target_heading < -0.7) { // crook to right, slow down left motor
                            if (lp > 0) lp *= 0.7;
                            else rp *= 0.7;
                        }
                    }
                }
                else if(robot.use_minibot) {
                    if (cur_heading - robot.target_heading > 0.7) { // crook to left,  slow down right motor
                        if (rp > 0) rp *= 0.7; //If the robot is going forward
                        else lp *= 0.7; // If the robot is going backwards
                    } else if (cur_heading - robot.target_heading < -0.7) { // crook to right, slow down left motor
                        if (lp > 0) lp *= 0.7;
                        else rp *= 0.7;
                    }
                }
            }
        }
        if(robot.use_swerve || robot.use_newbot) {
            if (robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(lp);
                    robot.motorBackRight.setPower(rp);
                }

            } else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB) {
                robot.motorFrontRight.setPower(lp);
                robot.motorFrontLeft.setPower(-lp);
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(-rp);
                    robot.motorBackRight.setPower(rp);
                }
            }
            else if(robot.cur_mode == SwerveDriveHardware.CarMode.TURN) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(lp);
                    robot.motorBackRight.setPower(rp);
                }
            }
        }
        else if (robot.use_minibot) {
            if (Math.abs(rp) > 0.3 && Math.abs(lp) > 0.3) {
                robot.motorFrontRight.setPower(rp * robot.DRIVE_RATIO_FR);
                robot.motorFrontLeft.setPower(lp * robot.DRIVE_RATIO_FL);
                if (!robot.use_minibot && !robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(lp * robot.DRIVE_RATIO_BL);
                    robot.motorBackRight.setPower(rp * robot.DRIVE_RATIO_BR);
                }
            } else {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                if (!robot.use_minibot && !robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(lp);
                    robot.motorBackRight.setPower(rp);
                }
            }
        }
    }

    public void driveTTCoast(double lp, double rp){
        boolean strafeRight = false;
        if(lp > 0 && rp > 0) {
            strafeRight = true;
        }
        else{
            strafeRight = false;
        }
        if(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
            if (!robot.use_front_drive_only) {
                robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
        else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB && strafeRight){
            if (!robot.use_front_drive_only) {
                robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            robot.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB && !strafeRight){
            robot.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (!robot.use_front_drive_only) {
                robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        if(!robot.fast_mode && robot.straight_mode) { // expect to go straight
            if (robot.use_imu) {
                double cur_heading = imu_heading();
                double heading_off_by = ((cur_heading - robot.target_heading) / 360);
                if(robot.use_swerve || robot.use_newbot) {
                    if(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                        if(rp > 0 && lp > 0) { //When going forward
                            if(robot.use_swerve) {
                                if (cur_heading - robot.target_heading > 0.7) {
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION - heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION - heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) {
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION + heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION + heading_off_by);
                                }
                            } else if(robot.use_newbot) {
                                if (cur_heading - robot.target_heading > 0.7) {
                                    robot.servoFrontLeft.setPosition(robot.NB_SERVO_FL_FORWARD_POSITION - heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) {
                                    robot.servoFrontLeft.setPosition(robot.NB_SERVO_FL_FORWARD_POSITION + heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                                }
                            }
                        }
                        else{ // When going backward
                            if(robot.use_swerve) {
                                if (cur_heading - robot.target_heading > 0.7) { //Drifting to the left
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION + heading_off_by); //Turn Front servos to the right
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION + heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) { //Drifting to the right
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION - heading_off_by); //Turn Front servos to the left
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION - heading_off_by);
                                }
                            } else if (robot.use_newbot) {
                                if (cur_heading - robot.target_heading > 0.7) { //Drifting to the left
                                    robot.servoFrontLeft.setPosition(robot.NB_SERVO_FL_FORWARD_POSITION + heading_off_by); //Turn Front servos to the right
                                    robot.servoFrontRight.setPosition(robot.NB_SERVO_FR_FORWARD_POSITION + heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) { //Drifting to the right
                                    robot.servoFrontLeft.setPosition(robot.NB_SERVO_FL_FORWARD_POSITION - heading_off_by); //Turn Front servos to the left
                                    robot.servoFrontRight.setPosition(robot.NB_SERVO_FR_FORWARD_POSITION - heading_off_by);
                                }
                            }
                        }
                    }
                    else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB){  //Tentative, could stand to remove
                        if (cur_heading - robot.target_heading > 0.7) { // crook to left,  slow down right motor
                            if (rp > 0) rp *= 0.7; //If the robot is going forward
                            else lp *= 0.7; // If the robot is going backwards
                        } else if (cur_heading - robot.target_heading < -0.7) { // crook to right, slow down left motor
                            if (lp > 0) lp *= 0.7;
                            else rp *= 0.7;
                        }
                    }
                }
                else if(robot.use_minibot) {
                    if (cur_heading - robot.target_heading > 0.7) { // crook to left,  slow down right motor
                        if (rp > 0) rp *= 0.7; //If the robot is going forward
                        else lp *= 0.7; // If the robot is going backwards
                    } else if (cur_heading - robot.target_heading < -0.7) { // crook to right, slow down left motor
                        if (lp > 0) lp *= 0.7;
                        else rp *= 0.7;
                    }
                }
            }
        }
        if(robot.use_swerve || robot.use_newbot) {
            if (robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(0);
                    robot.motorBackRight.setPower(0);
                }

            } else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB && strafeRight) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(0);
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(0);
                    robot.motorBackRight.setPower(lp);
                }
            }
            else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB && !strafeRight) {
                robot.motorFrontRight.setPower(0);
                robot.motorFrontLeft.setPower(rp);
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(lp);
                    robot.motorBackRight.setPower(0);
                }
            }
            else if(robot.cur_mode == SwerveDriveHardware.CarMode.TURN) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(lp);
                    robot.motorBackRight.setPower(rp);
                }
            }
        }
        else if (robot.use_minibot){
            if (Math.abs(rp) > 0.3 && Math.abs(lp) > 0.3) {
                robot.motorFrontRight.setPower(rp * robot.DRIVE_RATIO_FR);
                robot.motorFrontLeft.setPower(lp * robot.DRIVE_RATIO_FL);
                if (!robot.use_minibot && !robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(lp * robot.DRIVE_RATIO_BL);
                    robot.motorBackRight.setPower(rp * robot.DRIVE_RATIO_BR);
                }
            } else {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                if (!robot.use_minibot && !robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(lp);
                    robot.motorBackRight.setPower(rp);
                }
            }
        }
    }

    public void driveTTSnake(double drivePower, float turnIntensity, boolean snakeRight){ //Turn intensity is a value 0 to 1 meant to represent the triggers for determining the snake angle
        robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        calc_snake(snakeRight?0:turnIntensity,snakeRight?turnIntensity:0);
        snake_servo_adj();
        if(robot.use_newbot){
            robot.insideWheelsMod = drivePower * ((Math.pow((Math.pow(0.5 * robot.NB_LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) - robot.NB_WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                    (robot.r_Value));
            robot.outsideWheelsMod = drivePower * ((Math.pow((Math.pow(0.5 * robot.NB_LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) + robot.NB_WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                    (robot.r_Value));
        }
        if (!snakeRight) {
            robot.motorPowerLeft = robot.insideWheelsMod;
            robot.motorPowerRight = robot.outsideWheelsMod;
        } else {
            robot.motorPowerLeft = robot.outsideWheelsMod;
            robot.motorPowerRight = robot.insideWheelsMod;
        }
        robot.motorFrontRight.setPower(robot.motorPowerRight);
        robot.motorFrontLeft.setPower(robot.motorPowerLeft);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);
    }

    void stop_chassis() {
        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        if (!robot.use_minibot && !robot.use_front_drive_only) {
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);
        }
    }

    void reset_chassis()  {
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (!robot.use_minibot && !robot.use_front_drive_only) {
            robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        robot.leftCnt = 0;
        robot.rightCnt = 0;
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!robot.use_minibot&& !robot.use_front_drive_only) {
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void stop_auto() {
        if (robot.use_color_sensor) {
            if (!robot.use_newbot_v2) {
                robot.l_colorSensor.enableLed(false);
            }
            robot.r_colorSensor.enableLed(false);
            //robot.l_colorSensor.close();
            //robot.use_color_sensor = false;
        }
        if (robot.use_Vuforia) {
            robot.relicTrackables.deactivate();
            robot.use_Vuforia = false;
        }
        if (robot.use_camera) {
            robot.use_camera = false;
        }
        if(robot.use_newbot_v2 && robot.use_intake)
            intakeBarWheelStop();
    }

    void stop_tobot() {
        if (robot.use_swerve||robot.use_minibot||robot.use_newbot)
            stop_chassis();
        if (robot.use_color_sensor) {
            if (!robot.use_newbot_v2) {
                robot.l_colorSensor.enableLed(false);
                robot.l_colorSensor.close();
            }
            robot.r_colorSensor.enableLed(false);
            robot.r_colorSensor.close();
        }
        if (robot.sv_bar_wheel!=null) {
            robot.sv_bar_wheel.close();
        }
        // stop all sensors
    }

    public void run_until_encoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {
        robot.runtime.reset();
        int leftTC1 = leftCnt;
        int rightTC1 = rightCnt;
        int leftTC2 = 0;
        int rightTC2 = 0;
        int leftTC0 = 0;
        int rightTC0 = 0;
        int targetPosFrontLeft;
        int curPosFrontLeft = robot.motorFrontLeft.getCurrentPosition();
        int targetPosFrontRight;
        int curPosFrontRight = robot.motorFrontRight.getCurrentPosition();
        int targetPosBackLeft;
        int curPosBackLeft = (robot.motorBackLeft!=null?robot.motorBackLeft.getCurrentPosition():0);
        int targetPosBackRight;
        int curPosBackRight = (robot.motorBackRight!=null?robot.motorBackRight.getCurrentPosition():0);
        double initLeftPower = leftPower;
        double initRightPower = rightPower;
        double leftPowerSign = leftPower/Math.abs(leftPower);
        double rightPowerSign = rightPower/Math.abs(rightPower);
        boolean strafeRight = false;

        if(leftPower > 0 && rightPower > 0){
            strafeRight = true;
        }
        else{
            strafeRight = false;
        }

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (!robot.use_front_drive_only) {
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (Math.abs(leftPower) > 0.4 && leftTC1 > 600 && !robot.fast_mode) {
            leftTC2 = 150;
            leftTC0 = 75;
            leftTC1 -= 225;
        }
        if (Math.abs(rightPower) > 0.4 && rightTC1 > 600 && !robot.fast_mode) {
            rightTC2 = 150;
            rightTC0 = 75;
            rightTC1 -= 225;
        }
        if(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
            if (rightTC0 > 0 || leftTC0 > 0) {
                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC0);
                targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC0);
                robot.motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                robot.motorFrontRight.setTargetPosition(targetPosFrontRight);

                robot.runtime.reset();
                driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                while (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && (robot.runtime.seconds() < 1) && opModeIsActive()) {
                    driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                    // show_telemetry();
                }
            }
            curPosFrontLeft = robot.motorFrontLeft.getCurrentPosition();
            curPosFrontRight = robot.motorFrontRight.getCurrentPosition();

            targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC1);
            targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC1);

            robot.motorFrontLeft.setTargetPosition(targetPosFrontLeft);
            robot.motorFrontRight.setTargetPosition(targetPosFrontRight);

            driveTTCoast(leftPower, rightPower);
            int iter = 0;
            int prev_lpos = curPosFrontLeft;
            double cur_time = robot.runtime.nanoseconds()/1000000000.0;
            double prev_time = cur_time;
            double cur_speed = 0, prev_speed=0;
            while (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && (robot.runtime.seconds() < 7) && opModeIsActive()) {
                driveTTCoast(leftPower, rightPower);
                if ((++iter)%6==0 && robot.stop_on_dump==true) {
                    curPosFrontLeft = robot.motorFrontLeft.getCurrentPosition();
                    cur_time = robot.runtime.nanoseconds()/1000000000.0;
                    cur_speed = (curPosFrontLeft-prev_lpos) / (cur_time-prev_time);
                    if (iter>12 && Math.abs(cur_speed-prev_speed)<1.0 &&
                            Math.abs(cur_speed)<0.25 && Math.abs(curPosFrontLeft-targetPosFrontLeft)>40) {
                        robot.bump_detected = true;
                        break;
                    }
                    prev_lpos = curPosFrontLeft;
                    prev_time = cur_time;
                    prev_speed = cur_speed;
                }
                if (robot.use_verbose) {
                    telemetry.addData("4.Speed cur/prev/i=", "%.2f/%.2f/%1d", cur_speed, prev_speed, iter);
                    telemetry.addData("5.time cur/prev/^=", "%.4f/%.4f/%.4f", cur_time, prev_time, (cur_time-prev_time));
                    telemetry.addData("6.enco cur/prev/^=", "%2d/%2d/%2d", curPosFrontLeft, prev_lpos,(curPosFrontLeft-prev_lpos));
                    telemetry.update();
                }
                //if (robot.stop_on_dump && didBump()) {
                //    robot.bump_detected = true;
                //    break;
                //}
            }

            if (rightTC2 > 0 || leftTC2 > 0) {
                curPosFrontLeft = robot.motorFrontLeft.getCurrentPosition();
                curPosFrontRight = robot.motorFrontRight.getCurrentPosition();

                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC2);
                targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC2);

                robot.motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                robot.motorFrontRight.setTargetPosition(targetPosFrontRight);
                driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                while (!robot.bump_detected && robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && (robot.runtime.seconds() < 8) && opModeIsActive()) {
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                }
            }
            if (robot.use_verbose) {
                //stop_chassis();
                telemetry.addData("4.Speed cur/prev/i/bumped=", "%.2f/%.2f/%1d/%s",
                        cur_speed, prev_speed, iter, (robot.bump_detected?"T":"F"));
                telemetry.addData("5.time cur/prev/^=", "%.4f/%.4f/%.4f", cur_time, prev_time, (cur_time-prev_time));
                telemetry.addData("6.enco cur/prev/^=", "%2d/%2d/%2d", curPosFrontLeft, prev_lpos,(curPosFrontLeft-prev_lpos));
                telemetry.addLine("7.Hit B/X button to go next/exit ...");
                telemetry.update();
                //while (!gamepad1.x&&!gamepad1.b) {;}
            }
        }
        else if((robot.cur_mode == SwerveDriveHardware.CarMode.CRAB) && !robot.use_front_drive_only){
            if(strafeRight) {
                if (rightTC0 > 0 || leftTC0 > 0) {
                    targetPosFrontRight = curPosFrontRight + ((int) -leftPowerSign * leftTC0);
                    targetPosBackRight = curPosBackRight + ((int) -rightPowerSign * rightTC0);
                    robot.motorFrontRight.setTargetPosition(targetPosFrontRight);
                    robot.motorBackRight.setTargetPosition(targetPosBackRight);

                    robot.runtime.reset();
                    driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                    while (robot.motorFrontRight.isBusy() && robot.motorBackRight.isBusy() && (robot.runtime.seconds() < 1) && opModeIsActive()) {
                        driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                        // show_telemetry();
                    }
                }
                curPosFrontRight = robot.motorFrontRight.getCurrentPosition();
                curPosBackRight = robot.motorBackRight.getCurrentPosition();

                targetPosFrontRight = curPosFrontRight + ((int) -leftPowerSign * leftTC1);
                targetPosBackRight = curPosBackRight + ((int) -rightPowerSign * rightTC1);

                robot.motorFrontRight.setTargetPosition(targetPosFrontRight);
                robot.motorBackRight.setTargetPosition(targetPosBackRight);

                driveTTCoast(leftPower, rightPower);
                while (robot.motorFrontRight.isBusy() && robot.motorBackRight.isBusy() && (robot.runtime.seconds() < 7) && opModeIsActive()) {
                    driveTTCoast(leftPower, rightPower);
                }

                if (rightTC2 > 0 || leftTC2 > 0) {
                    curPosFrontRight = robot.motorFrontRight.getCurrentPosition();
                    curPosBackRight = robot.motorBackRight.getCurrentPosition();

                    targetPosFrontRight = curPosFrontRight + ((int) -leftPowerSign * leftTC2);
                    targetPosBackRight = curPosBackRight + ((int) -rightPowerSign * rightTC2);

                    robot.motorFrontRight.setTargetPosition(targetPosFrontRight);
                    robot.motorBackRight.setTargetPosition(targetPosBackRight);
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    while (robot.motorFrontRight.isBusy() && robot.motorBackRight.isBusy() && (robot.runtime.seconds() < 8) && opModeIsActive()) {
                        driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    }
                }
            }
            else{
                if (rightTC0 > 0 || leftTC0 > 0) {
                    targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC0);
                    targetPosBackLeft = curPosBackLeft + ((int) rightPowerSign * rightTC0);
                    robot.motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                    robot.motorBackLeft.setTargetPosition(targetPosBackLeft);

                    robot.runtime.reset();
                    driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                    while (robot.motorFrontLeft.isBusy() && robot.motorBackLeft.isBusy() && (robot.runtime.seconds() < 1) && opModeIsActive()) {
                        driveTTCoast(leftPowerSign * 0.3, rightPowerSign * 0.3);
                        // show_telemetry();
                    }
                }
                curPosFrontLeft = robot.motorFrontLeft.getCurrentPosition();
                curPosBackLeft = robot.motorBackLeft.getCurrentPosition();

                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC1);
                targetPosBackLeft = curPosBackLeft + ((int) rightPowerSign * rightTC1);

                robot.motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                robot.motorBackLeft.setTargetPosition(targetPosBackLeft);

                driveTTCoast(leftPower, rightPower);
                while (robot.motorFrontLeft.isBusy() && robot.motorBackLeft.isBusy() && (robot.runtime.seconds() < 7) && opModeIsActive()) {
                    driveTTCoast(leftPower, rightPower);
                }

                if (rightTC2 > 0 || leftTC2 > 0) {
                    curPosFrontLeft = robot.motorFrontLeft.getCurrentPosition();
                    curPosBackLeft = robot.motorBackLeft.getCurrentPosition();

                    targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC2);
                    targetPosBackLeft = curPosBackLeft + ((int) rightPowerSign * rightTC2);

                    robot.motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                    robot.motorBackLeft.setTargetPosition(targetPosBackLeft);
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    while (robot.motorFrontLeft.isBusy() && robot.motorBackLeft.isBusy() && (robot.runtime.seconds() < 8) && opModeIsActive()) {
                        driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                    }
                }
            }
        }
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!robot.use_front_drive_only) {
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        robot.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!robot.use_front_drive_only) {
            robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        stop_chassis();
        robot.runtime.reset();
    }

    boolean has_left_drive_encoder_reached(double p_count) {
        DcMotor mt = robot.motorFrontLeft;
        if(robot.use_swerve||robot.use_newbot){
            if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB){
                if (-robot.leftPower < 0) {
                    //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
                    return (mt.getCurrentPosition() <= p_count);
                } else {
                    //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
                    return (mt.getCurrentPosition() >= p_count);
                }
            }
            else{
                if (robot.leftPower < 0) {
                    //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
                    return (mt.getCurrentPosition() <= p_count);
                } else {
                    //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
                    return (mt.getCurrentPosition() >= p_count);
                }
            }
        }
        else {
            if (robot.leftPower < 0) {
                //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
                return (mt.getCurrentPosition() <= p_count);
            } else {
                //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
                return (mt.getCurrentPosition() >= p_count);
            }
        }
    } // has_left_drive_encoder_reached

    boolean has_right_drive_encoder_reached(double p_count) {
        DcMotor mt = robot.motorFrontRight;
        if (robot.rightPower < 0) {
            return (mt.getCurrentPosition() <= p_count);
        } else {
            return (mt.getCurrentPosition() >= p_count);
        }

    } // has_right_drive_encoder_reached

    boolean have_drive_encoders_reached(double p_left_count, double p_right_count) {
        DcMotor mt = robot.motorFrontRight;
        boolean l_return = false;
        if (has_left_drive_encoder_reached(p_left_count) && has_right_drive_encoder_reached(p_right_count)) {
            l_return = true;
        } else if (has_left_drive_encoder_reached(p_left_count)) { // shift target encoder value from right to left
            double diff = Math.abs(p_right_count - mt.getCurrentPosition()) / 2;
            if (robot.leftPower < 0) {
                robot.leftCnt -= diff;
            } else {
                robot.leftCnt += diff;
            }
            if (robot.rightPower < 0) {
                robot.rightCnt += diff;
            } else {
                robot.rightCnt -= diff;
            }
        } else if (has_right_drive_encoder_reached(p_right_count)) { // shift target encoder value from left to right
            double diff = Math.abs(p_left_count - mt.getCurrentPosition()) / 2;
            if (robot.rightPower < 0) {
                robot.rightCnt -= diff;
            } else {
                robot.rightCnt += diff;
            }
            if (robot.leftPower < 0) {
                robot.leftCnt += diff;
            } else {
                robot.leftCnt -= diff;
            }
        }
        return l_return;
    } // have_encoders_reached

    void StraightR(double power, double n_rotations) throws InterruptedException {
        robot.straight_mode = true;
        reset_chassis();
        // set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = robot.motorFrontLeft.getCurrentPosition();
        int rightEncode = robot.motorFrontRight.getCurrentPosition();
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        robot.leftCnt = (int) (robot.ONE_ROTATION * n_rotations);
        robot.rightCnt = (int) (robot.ONE_ROTATION * n_rotations);
        robot.leftPower = robot.rightPower = (float) power;
        run_until_encoder(robot.leftCnt, robot.leftPower, robot.rightCnt, robot.rightPower);
        robot.straight_mode = false;
        if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB) {
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_STRAFE_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_STRAFE_POSITION);
            robot.servoBackLeft.setPosition(robot.SERVO_BL_STRAFE_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_STRAFE_POSITION);
        } else {
            robot.set_chassis_forward_position();
        }

        //if (!robot.fast_mode)
        //    sleep(135);
    }

    void StraightIn(double power, double in) throws InterruptedException {
        if (robot.use_imu) {
            robot.target_heading = imu_heading();
        }
        if (robot.use_encoder) {
            double numberR = in / robot.INCHES_PER_ROTATION;
            if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB){
                StraightR(power, numberR);
            }
            else {
                StraightR(-power, numberR);
            }
        } else { // using timer
            double in_per_ms = 0.014 * power / 0.8;
            if (in_per_ms < 0) in_per_ms *= -1.0;
            long msec = (long) (in / in_per_ms);
            if (msec < 100) msec = 100;
            if (msec > 6000) msec = 6000; // limit to 6 sec
            driveTT(power, power);
            sleep(msec);
            driveTT(0, 0);
        }
    }

    void StraightCm(double power, double cm) throws InterruptedException {
        if (robot.use_imu) {
            robot.target_heading = imu_heading();
        }
        if (robot.use_encoder) {
            double WHEEL_DIAMETER = 102.18; // In millimeters
            double WHEEL_RADIUS = WHEEL_DIAMETER / 2; // Still in millimeters
            double CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // Also is mm per rotation

            double numberR = cm / (CIRCUMFERENCE / 10);
            if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB){
                StraightR(power, numberR);
            }
            else {
                StraightR(-power, numberR);
            }
        } else { // using timer
            double cm_per_ms = 0.014 * power / 0.8;
            if (cm_per_ms < 0) cm_per_ms *= -1.0;
            long msec = (long) (cm / cm_per_ms);
            if (msec < 100) msec = 100;
            if (msec > 6000) msec = 6000; // limit to 6 sec
            driveTT(power, power);
            sleep(msec);
            driveTT(0, 0);

        }
    }

    void crabRight(double power) {
        if (robot.old_mode!=SwerveDriveHardware.CarMode.CRAB) {
            change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
            sleep(200);
        }
        driveTT(-Math.abs(power), -Math.abs(power)); // Crabs to the right
    }

    void crabLeft(double power) {
        if (robot.old_mode!=SwerveDriveHardware.CarMode.CRAB) {
            change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
            sleep(200);
        }
        driveTT(Math.abs(power), Math.abs(power)); // Crabs to the right
    }

    void TurnRightD(double power, double degree) throws InterruptedException {

        double adjust_degree_imu = robot.IMU_ROTATION_RATIO_R * degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        robot.runtime.reset();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = robot.motorFrontLeft.getCurrentPosition();
        int rightEncode = robot.motorFrontRight.getCurrentPosition();
        robot.leftCnt = (int) (robot.ONE_ROTATION * robot.RROBOT * degree / 720.0);
        robot.rightCnt = (int) (-robot.ONE_ROTATION * robot.RROBOT * degree / 720.0);

        robot.leftPower = (float) -power;
        robot.rightPower = (float) power;

        change_swerve_pos(SwerveDriveHardware.CarMode.TURN);

        sleep(100);

        robot.leftCnt += leftEncode;
        robot.rightCnt += rightEncode;

        //DbgLog.msg(String.format("imu Right Turn %.2f degree with %.2f power.", degree, power));
        if (robot.use_imu) {
            current_pos = imu_heading();
            robot.target_heading = current_pos - adjust_degree_imu;
            if (robot.target_heading <= -180) {
                robot.target_heading += 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos <= 0)) {
                current_pos += 360;
            }
            while ((current_pos >= robot.target_heading) && (robot.runtime.seconds() < 4.0) && opModeIsActive()) {
                current_pos = imu_heading();
                // DbgLog.msg(String.format("imu current/target heading = %.2f/%.2f",current_pos,target_heading));

                if (heading_cross_zero && (current_pos <= 0)) {
                    current_pos += 360;
                }
                driveTT(robot.leftPower, robot.rightPower);
            }
        } else {
            if (robot.use_encoder) {
                run_until_encoder(robot.leftCnt, robot.leftPower, robot.rightCnt, robot.rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(robot.leftPower, robot.rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
        if (!opModeIsActive()) return;
        sleep(100);
        change_swerve_pos(robot.old_mode);
        //if (!robot.fast_mode)
        //    sleep(135);
    }

    void TurnLeftD(double power, double degree) throws InterruptedException {
        double adjust_degree_imu = robot.IMU_ROTATION_RATIO_L * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        robot.runtime.reset();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS)
        int leftEncode = robot.motorFrontLeft.getCurrentPosition();
        int rightEncode = robot.motorFrontRight.getCurrentPosition();
        robot.leftCnt = (int) (-robot.ONE_ROTATION * robot.RROBOT * degree / 720.0);
        robot.rightCnt = (int) (robot.ONE_ROTATION * robot.RROBOT * degree / 720.0);

        robot.leftPower = (float) power;
        robot.rightPower = (float) -power;

        change_swerve_pos(SwerveDriveHardware.CarMode.TURN);

        sleep(100);

        robot.leftCnt += leftEncode;
        robot.rightCnt += rightEncode;


        //DbgLog.msg(String.format("imu Left Turn %.2f degree with %.2f power.", degree, power));
        if (robot.use_imu) {
            current_pos = imu_heading();
            robot.target_heading = current_pos + adjust_degree_imu;
            if (robot.target_heading >= 180) {
                robot.target_heading -= 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos >= 0)) {
                current_pos -= 360;
            }
            //DbgLog.msg(String.format("imu Left Turn curr/target pos = %.2f/%.2f.", current_pos, target_heading));
            while ((current_pos <= robot.target_heading) && (robot.runtime.seconds() < 5.0) && opModeIsActive()) {
                current_pos = imu_heading();
                if (heading_cross_zero && (current_pos >= 0)) {
                    current_pos -= 360;
                }
                driveTT(robot.leftPower, robot.rightPower);
            }
        } else {
            if (robot.use_encoder) {
                run_until_encoder(robot.leftCnt, robot.leftPower, robot.rightCnt, robot.rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(robot.leftPower, robot.rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }
        }
        driveTT(0, 0);
        sleep(100);
        change_swerve_pos(robot.old_mode);
        //if (!robot.fast_mode)
        //    sleep(135);
    }

    public int get_cryptobox_column() throws InterruptedException {

        int column = -1;
        if (!robot.use_Vuforia)
            return column;

        robot.relicTrackables.activate();
        robot.runtime.reset();
        while (robot.runtime.seconds() < 2.0 && column == -1) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
                column = getColumnIndex(vuMark);
            }
        }
        return column;
    }

    double calcDelta(boolean isBlueAlliance) throws InterruptedException {
        if(!robot.use_color_sensor) {
            return 0;
        }
        if (robot.use_newbot) {
            if (isBlueAlliance || robot.use_newbot_v2) {
                robot.blue = robot.r_colorSensor.blue();
                robot.red = robot.r_colorSensor.red();
            } else {
                robot.blue = robot.l_colorSensor.blue();
                robot.red = robot.l_colorSensor.red();
            }
        } else {
            if (isBlueAlliance) {
                robot.blue = robot.l_colorSensor.blue();
                robot.red = robot.l_colorSensor.red();
            } else {
                robot.blue = robot.r_colorSensor.blue();
                robot.red = robot.r_colorSensor.red();
            }
        }
        return (robot.blue - robot.red);
    }

    /**
     * Function to prevent range sensor from returning an error
     * If this returns 999, it failed to get the right range in under .3 seconds
     * @param direction
     * @return
     */
    double getRange(RangeSensor direction){
        ElapsedTime elapsedTime = new ElapsedTime();
        double distance = 999;
        if (!robot.use_range_sensor)
            return 0.0;
        if(direction == RangeSensor.FRONT_LEFT){
            if (robot.rangeSensorFrontLeft==null)
                distance = 0;
            else while(distance > 365 && elapsedTime.seconds() < 0.3){
                distance = robot.rangeSensorFrontLeft.getDistance(DistanceUnit.CM);
            }
        } else if(direction == RangeSensor.FRONT_RIGHT){
            if (robot.rangeSensorFrontRight==null)
                distance = 0;
            else while(distance > 365 && elapsedTime.seconds() < 0.3){
                distance = robot.rangeSensorFrontRight.getDistance(DistanceUnit.CM);
            }
        } else if(direction == RangeSensor.BACK){
            if (robot.rangeSensorBack==null)
                distance = 0;
            else while(distance > 365 && elapsedTime.seconds() < 0.3){
                distance = robot.rangeSensorBack.getDistance(DistanceUnit.CM);
            }
        }
        else {
            throw new IllegalArgumentException("Direction not specified!");
        }
        if (distance>365) distance = 999;
        return distance;
    }

    /**
     * doPlatformMission is meant to execute everything that needs to be done on the starting platform, including
     * determining what the bonus column is for the cryptobox from the pictograph, determining the jewel colors and
     * knocking off the opposing alliance ball based on the colors from the color class.
     * @param isBlueAlliance Used to determine which ball to hit
     * @throws InterruptedException
     */
    public double doPlatformMission(boolean isBlueAlliance) throws InterruptedException {
        // return distance to be compensate

        if (!opModeIsActive()) return 0;

        if (robot.targetColumn<0) {
            robot.targetColumn = get_cryptobox_column();
        }

        if (robot.use_newbot) {
            if (robot.allianceColor == TeamColor.BLUE || robot.use_newbot_v2) {
                r_arm_down();
            } else {
                l_arm_down();
            }
        } else { // oldBot
            if (robot.allianceColor == TeamColor.BLUE) {
                l_arm_down();
            } else {
                r_arm_down();
            }
        }

        if(robot.use_camera) {
                new Thread(new Runnable() {
                    //These constants are for setting a selected portion of the image from Camera
                    //(Assuming portrait) Top left is (0,0), Top right (0,1), Bottom left is (1,0), Bottom right is (1,1)
                    double IMAGE_WIDTH_CROP = 1;
                    double IMAGE_HEIGHT_CROP = 1;
                    double IMAGE_OFFSET_X = 0; // Cannot be 1, make sure take the respective crop into consideration
                    double IMAGE_OFFSET_Y = 0; // Cannot be 1, make sure take the respective crop into consideration

                    ElapsedTime runTime = new ElapsedTime();

                    @Override
                    public void run() {
                        robot.camera.activate();

                        do{
                            robot.bitmap = robot.camera.captureBitmap(IMAGE_OFFSET_X, IMAGE_OFFSET_Y, IMAGE_WIDTH_CROP, IMAGE_HEIGHT_CROP);
                        }
                        while (runTime.seconds() < 0.5 && robot.bitmap == null);


                        if (robot.bitmap != null) {
                            robot.leftJewelColorCamera = determineJewelColor(robot.bitmap);
                            telemetry.addData("Left Jewel Color", robot.leftJewelColorCamera);
                            robot.rightJewelColorCamera = robot.leftJewelColorCamera.getOpposingColor();
                            robot.camReady = true;
                            robot.camera.stopCamera();
                        }
                    }
                }).start();
        }

        if (!opModeIsActive()) return 0;

        if (robot.use_glyph_grabber) {
            glyph_grabber_close();
            sleep(100);
            glyph_slider_up_inches(.5, 3);
        }

        if (robot.use_newbot_v2)
            intakeGateUp();

        if (!opModeIsActive()) return 0;

        sleep(500);
        if (!opModeIsActive()) return 0;

        TeamColor rightJewelColorCS = checkBallColor(isBlueAlliance);

        // Comparing of sensor data
        boolean isRedBall = (rightJewelColorCS == TeamColor.RED);
        boolean isBlueBall = (rightJewelColorCS == TeamColor.BLUE);

        telemetry.addData("isBlueBall/isRedBall", "%s/%s", isBlueBall, isRedBall);
        telemetry.update();
        // sleep(2000);
        double next_dist = 0;

        //gives time for camera if thread isn't done
        if (!robot.camReady) {
            telemetry.addData("Camera wasn't ready on time!", null).setRetained(true);
            sleep(100);
        }

        // Determines if right jewel is red
        int directionI = calcArmDirection(rightJewelColorCS, robot.rightJewelColorCamera, isBlueAlliance);
        if (robot.use_newbot_v2 && !isBlueAlliance) {
            directionI *= -1;
        }
        if (!opModeIsActive()) return 0;
        if (robot.sv_jkicker!=null) {
            if (directionI<0)
                jkick_left();
            else if (directionI>0)
                jkick_right();
            r_arm_up();
        } else if (robot.use_newbot) {
            int dist = (directionI > 0 ? 7 : 6);
            StraightCm(-.2 * directionI, dist); // Drives forward if right jewel is red, backwards if blue
            sleep(100);
            if (isBlueAlliance || robot.use_newbot_v2)
                r_arm_up(); // arm up to ensure the jewel is knocked out
            else
                l_arm_up(); // arm up to ensure the jewel is knocked out
            // next_dist = -1.0*dist*directionI;
            StraightCm(.4 * directionI, dist); // Drives forward if right jewel is blue, backwards if red
        } else { // oldBot
            if (isBlueAlliance) {
                if (directionI == 1) { // Right jewel is our color
                    arm_left();
                } else if (directionI == -1) { // Right jewel is their color
                    arm_right();
                }
                // if directionI 0, then color is unknown
            } else {
                int dist = (directionI > 0 ? 7 : 5);
                StraightCm(.1 * directionI, dist); // Drives forward if right jewel is red, backwards if blue
                sleep(100);
                r_arm_up(); // arm up to ensure the jewel is knocked out
                StraightCm(-.15 * directionI, dist); // Drives forward if right jewel is blue, backwards if red
            }
        }
        if (!opModeIsActive()) return next_dist;

        if (isBlueAlliance && !robot.use_newbot) {
            sleep(500);
            l_arm_up();
        }
        if (robot.use_verbose)
            telemetry.addData("0: End doPlatformMission CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());

        return next_dist;
    }

    // Provided two colors guesses and team color, determines if the right ball is the same color as our team.
    // if sum = 1, then it is our color. if sum = 0, then it is unknown. if sum = -1, then it is their color.
    // Ex. sensor is red, camera is unknown, team is red. It will return 1.
    public int calcArmDirection(TeamColor sensor, TeamColor camera, boolean isBlueAlliance) {
        int result = 0;
        if (sensor == TeamColor.RED)
            result = 1;
        else if (sensor == TeamColor.BLUE)
            result = -1;
        else { // Sensor trumps camera, only use camera when sensor is unknown
            if (camera == TeamColor.RED)
                result = 1;
            else if (camera == TeamColor.BLUE)
                result = -1;
        }
//        if (isBlueAlliance)
//            result *= -1;
//        if (robot.use_newbot && isBlueAlliance){
//            result *= -1;
//        }
        return result;
    }

    void enable_bump_detection() {
        robot.bump_detected = false;
        robot.stop_on_dump = true;
    }

    void disable_bump_detection() {
        robot.bump_detected = false;
        robot.stop_on_dump = false;
    }

    // returns true if it bumped the wall (noticeable negative acceleration), false if it went two seconds without hitting the wall
    public boolean didBump() {
        robot.accel = robot.imu.getAcceleration();
        if (Math.abs(robot.accel.xAccel)+Math.abs(robot.accel.zAccel) >= 1.8) {
            robot.bump_detected = true;
            return true;
        }
        return false;
    }

    public boolean GlyphStuck() {
        if (robot.rangeSensorBack==null)
            return false;
        return (getRange(RangeSensor.BACK)<5.1);
    }

    public void grabAndDump(boolean isSide, boolean isBlue) throws InterruptedException {
        intakeGateMid();
        double orig_imu = imu_heading();

        if (opModeIsActive()==false ||
                (isSide==true && robot.runtimeAuto.seconds() > 24) ||
                (isSide==false && robot.runtimeAuto.seconds() > 22)) {
            return;
        }
        if (opModeIsActive()) {
            robot.fast_mode = true;
            for(int i=0; i<2; i++) {
                double distance = getRange(RangeSensor.BACK);
                if (distance>20) {
                    distance += 20;
                    StraightCm((i==0?0.95:0.3), distance);
                }
            }
            robot.fast_mode = false;
        }
        boolean got_one = autoIntakeGlyphs(isSide, isBlue);

        if(isSide) alignUsingIMU(0.3, orig_imu);
        else alignUsingIMU(0.3, orig_imu);

        if (opModeIsActive()) {
            enable_bump_detection();
            for (int i=0; i<2 && robot.bump_detected==false; i++) {
                double dist = (isSide? Math.max(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 20:
                        Math.min(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 20);
                if (i==0) {
                    if (isSide) {
                        if (dist < 53) dist = 53;
                        else if (dist > 80)
                            dist = 80;
                    } else { // front
                        if (dist < 93) dist = 93;
                        else if (dist > 120)
                            dist = 120;
                    }
                    if (robot.runtimeAuto.seconds() > 27.5 || !got_one) {
                        dist -= 15;
                    }
                } else if (dist<1) break;
                if (i==0)robot.fast_mode = true;
                StraightCm((i==0?-0.9:-0.5), dist);
                robot.fast_mode =false;
                if (robot.runtimeAuto.seconds() > 29) return;
            }
            if (robot.bump_detected) {
                StraightCm(0.6,5);
            }
            disable_bump_detection();
            StraightCm(0.6, 4);
        }
        if((robot.runtimeAuto.seconds() > 29 || !got_one) && !robot.servo_tune_up){
            return;
        }
        else if (got_one && opModeIsActive()) {
            //lift_up_level_half();
            quickDump(isSide);
            //lift_back_init();
//            if (alignBoxEdge() && opModeIsActive()) {
//                deliverGlyph();
//            }
        }
        if (robot.use_verbose)
            telemetry.addData("0: End GrabAndDump() CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
    }

    public void quickDump(boolean isSide) throws InterruptedException {
        double turn_left_angles = -5;
        if (!opModeIsActive()) return;
        dumper_vertical();
        if (isSide) {
            if (robot.targetColumn == 0) {
                TurnRightD(0.6, 5);
            } else {
                TurnLeftD(0.6, 5);
            }
            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
        }
        else {
            if (robot.allianceColor == TeamColor.RED) {
                TurnLeftD(0.6, 3);
            } else {
                TurnRightD(0.6, 3);
            }
            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
        }
        // dumper_up();
        if (!opModeIsActive()) return;
        sleep(300);
        if (!opModeIsActive()) return;
        if (robot.runtimeAuto.seconds() < 29 || robot.servo_tune_up==true) {
            // sleep(100);
            driveTT(0.6, 0.6); // drive backward for .2 sec
            sleep(300);
            driveTT(0, 0);
            if (!opModeIsActive()) return;
            StraightIn(0.9,3); // out 5.5 in
            if (!opModeIsActive()) return;
            if (robot.runtimeAuto.seconds() < 29.5)
                sleep(100);
            StraightIn(0.9,2.5); // out 5.5 in
            if (!opModeIsActive()) return;
            dumper_down(false);
        } else {
            StraightIn(0.9,5.5);
        }
        if (!opModeIsActive()) return;
    }

    public boolean autoIntakeGlyphs(boolean isSide, boolean isBlue) throws InterruptedException {
        boolean got_at_least_one = false;
        boolean got_two = false;
        boolean tried_two = false;
        double time_out = (isSide?20:19);
        reset_prox();
        for (int i=0; i<1; i++) {
            // StraightIn(0.2,6);
            got_at_least_one = autoIntakeOneGlyph(isSide, isBlue);
        }
        if(robot.runtimeAuto.seconds() < time_out-4 && got_at_least_one && !gotTwoGlyphs()) {
            robot.snaked_left = false;
            got_two = autoIntakeSecondGlyph(isSide, isBlue);
        }
        if(robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false) return got_at_least_one;
        if (got_at_least_one && opModeIsActive()) {
            //dumper_shake();
            //intakeIn();
            //sleep(100);
            //intakeStop();
            //dumper_shake();
            robot.sv_dumper.setPosition(robot.SV_DUMPER_LIFT);
        }
        return got_at_least_one;
    }

    public boolean autoIntakeOneGlyph(boolean isSide, boolean isBlue) throws InterruptedException {
        double time_out = (isSide?26:24.5);

        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
            return false;
        }
        boolean got_one = autoIntake(isSide, true, isBlue);
        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
            return false;
        }

        for (int i=0; (i<3) && !got_one; i++) { // try upto 3 times
            if(robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false) return false;
            got_one = autoIntake(isSide, true, isBlue);
            if (!opModeIsActive()) return false;
        }
        return got_one;
    }

    public boolean autoIntakeSecondGlyph(boolean isSide,boolean isBlue) throws InterruptedException {
        double time_out = (isSide?22:20);
        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
            return false;
        }
        StraightCm(0.4,15);
        boolean got_two = autoIntake(isSide, false, isBlue);
        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
            return false;
        }
        for (int i=0; (i<1) && !got_two; i++) { // try upto 1 time
            if(robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false) return false;
            got_two = autoIntake(isSide, false, isBlue);
            if (!opModeIsActive()) return false;
        }

        return got_two;
    }

    public boolean autoIntake(boolean isSide, boolean isFirstGlyph, boolean isBlue) throws InterruptedException {
        double time_out = (isSide?26:24.5);
        boolean got_one = false;
        boolean curve_right = robot.snaked_left;
        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > (time_out-1) && robot.servo_tune_up==false)) {
            intakeStop(); stop_chassis();return false;
        }
//        double cur_heading = imu_heading();
//        if (isBlue) { // drive to right
//            driveTT(-0.2,-0.1);
//        } else {
//            driveTT(-0.1,-0.2);
//        }
        driveTTSnake(-0.5,(float) 1.0,curve_right);
        robot.snaked_left = !robot.snaked_left;
        intakeIn();
        sleep(700);
        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > (time_out-.5) && robot.servo_tune_up==false)) {
            intakeStop(); stop_chassis();return false;
        }
        stop_chassis();
        if(GlyphStuck()) {
            if(!robot.tried_clockwise) {
                correctGlyph(false);
                robot.tried_clockwise = true;
            }
            else{
                correctGlyph(false);
                robot.tried_clockwise = false;
            }
        }
        else{
            intakeOut();
            sleep(300);
        }
        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
            intakeStop(); stop_chassis();return false;
        }
        //driveTTSnake(-0.3,(float) 1.0,!curve_right);
        intakeIn();
        sleep(600);
        intakeStop();
        stop_chassis();
        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > (time_out+0.5) && robot.servo_tune_up==false)) {
            return false;
        }
        if(isFirstGlyph) {
            got_one = gotOneGlyph();
        }
        else{
            got_one = gotTwoGlyphs();
        }
        return got_one;
    }

    boolean gotOneGlyph() {
        boolean got_one=false;
        if (robot.use_proximity_sensor) {
            got_one = !robot.proxFL.getState() || !robot.proxML.getState();
        }
        return got_one;
    }

    boolean gotTwoGlyphs() {
        boolean got_two=false;
        if (robot.use_proximity_sensor) {
            got_two = !robot.proxML.getState() && !robot.proxFL.getState();
        }
        return got_two;
    }

    boolean alignBoxEdge() throws InterruptedException {
        if (robot.old_mode!=SwerveDriveHardware.CarMode.CAR) {
            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
            sleep(300);
        }
        double dist = Math.max(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 18;
        if (dist>0) StraightCm(-.2, dist);
        if (!robot.servo_tune_up) {
            if (robot.allianceColor == TeamColor.BLUE) {
                alignUsingIMU(0.18, 90);
            } else {
                alignUsingIMU(0.18, -90);
            }
        }
        dist = Math.max(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 16;
        if (dist>0) StraightCm(-.2, dist);
        change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
        reset_prox();
        boolean edge_detected_L = robot.proxL.getState();
        boolean edge_detected_R = robot.proxR.getState();
        boolean aligned = (edge_detected_L && edge_detected_R);
        boolean no_dump = false;
        if (!aligned) { // done
            if (!edge_detected_L) {
                crabRight(0.2);
            } else {
                crabLeft(0.2);
            }
        }
        if (!opModeIsActive()) return false;
        robot.runtime.reset();
        do {
            edge_detected_L = robot.proxL.getState();
            edge_detected_R = robot.proxR.getState();
            aligned = (edge_detected_L && edge_detected_R);
            if (!opModeIsActive()) no_dump = true;
        } while (!aligned && !no_dump && (robot.runtime.seconds() < 2));
        driveTT(0, 0); // Stops
        if (!opModeIsActive()) return false;

        change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
        sleep(300);
        StraightCm(.2, 3); // goes back so that delivery has enough space
        return aligned;
    }

    public void deliverGlyph() throws InterruptedException{
        if (!opModeIsActive()) return;
        if (robot.use_newbot) {
            StraightIn(0.6, 2);
            if (!opModeIsActive()) return;
            dumper_vertical();
            // dumper_up();
            if (!opModeIsActive()) return;
            sleep(500);
            if (!opModeIsActive()) return;
            driveTT(0.4, 0.4); // drive backward for .3 sec
            sleep(250);
            driveTT(0,0);
            if (!opModeIsActive()) return;
            StraightIn(0.8, 6);
            dumper_down(true);
        } else {
            StraightIn(0.3, 7);
            if (!opModeIsActive()) return;
            glyph_grabber_open_and_push_auto();
            // glyph_grabber_half_close();
            if (!opModeIsActive()) return;
            StraightIn(-0.7, 7);
            if (!opModeIsActive()) return;
            rotate_refine(); // ensure grabber back straight
            if (!opModeIsActive()) return;
            glyph_slider_back_init();
            if (!opModeIsActive()) return;
            // glyph_grabber_close();
            // sleep(500);
            // 0.4 will break the arms
            StraightIn(0.3, 10);
            if (!opModeIsActive()) return;
            //sleep(100);
            StraightIn(-0.7, 3);
            if (!opModeIsActive()) return;
            glyph_grabber_auto_open();
        }
        if (robot.use_verbose)
            telemetry.addData("0: End deliveryGlyph() CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
    }

    public void turnToCenter(boolean isBlue, boolean isSide, int curColumn) throws InterruptedException{
        if (!opModeIsActive()) return;
        if (robot.use_newbot) {
            if (isSide) {
                if (isBlue) {
                    TurnLeftD(.5,5);
                }
                else {
                    TurnRightD(.5,5);
                }
            }
            else {
                if (isBlue) {
                    TurnLeftD(0.5, 20);
                }
                else {
                    TurnRightD(.5, 20);
                }
            }
        }
        else {
            StraightCm(-0.3, 6);
            if (!opModeIsActive()) return;
            if (isSide) {
                if (isBlue) {
                    TurnRightD(0.6, 175);
                } else {
                    TurnLeftD(0.6, 175);
                }
            } else {
                if (isBlue) {
                    if (curColumn == 0) {
                        TurnRightD(0.6, 125);
                    } else if (curColumn == 1) {
                        TurnRightD(0.6, 135);
                    } else if (curColumn == 2) {
                        TurnRightD(0.6, 155);
                    }
                } else {
                    if (curColumn == 0) {
                        TurnLeftD(0.6, 155);
                    } else if (curColumn == 1) {
                        TurnLeftD(0.6, 135);
                    } else if (curColumn == 2) {
                        TurnLeftD(0.6, 125);
                    }
                }
            }
        }
    }

    TeamColor checkBallColor(boolean isBlueAlliance) throws InterruptedException {
        boolean isBlueBall = false;
        boolean isRedBall = false;
        robot.runtime.reset();
        double d = calcDelta(isBlueAlliance);
        TeamColor result = TeamColor.UNKNOWN;

        while (!isBlueBall && !isRedBall && (robot.runtime.seconds()<1.5)) {
            if ((d >= robot.BLUE_BALL_MIN) && (d <= robot.BLUE_BALL_MAX)) {
                isBlueBall = true;
            } else {
                isBlueBall = false;
            }
            if (d >= robot.RED_BALL_MIN && d <= robot.RED_BALL_MAX) {
                isRedBall = true;
            } else {
                isRedBall = false;
            }

            d = calcDelta(isBlueAlliance);
        }
//        telemetry.addData("delta/isBlueBall/isRedBall=", "%3.1f/%s/%s",d,robot.isBlueBall,robot.isRedBall);
//        telemetry.update();
        if (isBlueBall && isRedBall) {
            result = TeamColor.UNKNOWN;
        } else if (isBlueBall) {
            result = TeamColor.BLUE;
        }
        else if (isRedBall) {
            result = TeamColor.RED;
        }
        else {
            result = TeamColor.UNKNOWN;
        }
        return result;
    }

    int getColumnIndex(RelicRecoveryVuMark vuMark) throws InterruptedException {
        // return row index for Cryptograph
        // unknown : -1
        // left    :  0
        // center  :  1
        // right   :  2
        if (vuMark == RelicRecoveryVuMark.LEFT)
            return 0;
        else if (vuMark == RelicRecoveryVuMark.CENTER)
            return 1;
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
            return 2;

        return -1;
    }

    void l_arm_up() {
        if (robot.use_newbot) {
            robot.sv_left_arm.setPosition(robot.SV_LEFT_ARM_UP_NB);
        } else {
            robot.sv_elbow.setPosition(robot.SV_ELBOW_UP);
            sleep(200);
            robot.sv_shoulder.setPosition(robot.SV_SHOULDER_INIT);
        }
}

    void l_arm_down() {
        if (robot.use_newbot) {
            robot.sv_left_arm.setPosition(robot.SV_LEFT_ARM_DOWN_NB);
        } else {
            robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
            robot.sv_shoulder.setPosition(robot.SV_SHOULDER_DOWN);
        }
    }

    void r_arm_up() {
        if (robot.sv_jkicker!=null) {
            jkick_up();
        }
        if (robot.use_newbot) {
            robot.sv_right_arm.setPosition(robot.SV_RIGHT_ARM_UP_NB);
        } else {
            robot.sv_right_arm.setPosition(robot.SV_RIGHT_ARM_UP);
        }
        sleep(200);

    }

    void r_arm_down() {
        if (robot.use_newbot) {
            robot.sv_right_arm.setPosition(robot.SV_RIGHT_ARM_DOWN_NB);
        } else{
            robot.sv_right_arm.setPosition(robot.SV_RIGHT_ARM_DOWN);
        }
        sleep(200);
    }

    void jkick_right() {
        if (robot.sv_jkicker==null) return;
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_RIGHT1);
        sleep(100);
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_RIGHT2);
        sleep(150);
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_RIGHT);
        sleep(150);
    }

    void jkick_left() {
        if (robot.sv_jkicker==null) return;
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_LEFT1);
        sleep(100);
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_LEFT2);
        sleep(150);
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_LEFT);
        sleep(150);
    }

    void jkick_up() {
        if (robot.sv_jkicker==null) return;
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_UP);
        sleep(300);
    }

    void arm_left() {
        robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN_HIT);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_LEFT_1);
        sleep(500);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_LEFT_2);
        sleep(500);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_LEFT_3);
    }

    void arm_right() {
        robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN_HIT);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_RIGHT_1);
        sleep(500);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_RIGHT_2);
        sleep(500);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_RIGHT_3);
    }

    void front_arm_in() {
        if (!robot.use_front_arm)
            return;
        robot.sv_front_arm.setPosition(robot.SV_FRONT_ARM_IN);
    }

    void front_arm_out() {
        if (!robot.use_front_arm)
            return;
        robot.sv_front_arm.setPosition(robot.SV_FRONT_ARM_OUT);
    }

    void front_arm_sweep() {
        stop_chassis();
        front_arm_out();
        sleep(300);
        front_arm_in();
    }


    void reset_prox(){
        robot.proxL.setMode(DigitalChannel.Mode.OUTPUT);
        robot.proxR.setMode(DigitalChannel.Mode.OUTPUT);
        robot.proxFL.setMode(DigitalChannel.Mode.OUTPUT);
        robot.proxML.setMode(DigitalChannel.Mode.OUTPUT);
        robot.proxL.setState(true);
        robot.proxR.setState(true);
        robot.proxFL.setState(true);
        robot.proxML.setState(true);
        robot.proxL.setMode(DigitalChannel.Mode.INPUT);
        robot.proxR.setMode(DigitalChannel.Mode.INPUT);
        robot.proxFL.setMode(DigitalChannel.Mode.INPUT);
        robot.proxML.setMode(DigitalChannel.Mode.INPUT);
        sleep(200);
    }

    void go_to_crypto(double next_dist, double power, int targetColumn, boolean isBlue, boolean isSideBox)throws InterruptedException {
        if (targetColumn < 0) targetColumn = 1;
        boolean nearest_col = false;
        if ((targetColumn==0 && isBlue==true) || (targetColumn==2 && isBlue==false))
            nearest_col = true;

        int direction = (robot.use_newbot_v2&&!isBlue?1:-1);
        if (robot.use_newbot) {
            double dist = next_dist + (isSideBox ? 20 : 24);
            if (nearest_col)
                dist += 3;
            StraightIn(direction*.3, dist); // Drive off the balance stone
        } else {
            StraightIn(-.25, (isSideBox ? 22 : 24)); // Drive off the balance stone
        }
        if (direction==1 && !isSideBox) { // red front need to turn 180 degree
            TurnLeftD(0.4, 180);
            alignUsingIMU(0.18, 178.0);
        } else {
            alignUsingIMU(0.18, 0);
        }

        double driveDistance;
        double dist;

        if (isSideBox) {
            if (isBlue) driveDistance = 6 + (18.5 * targetColumn); // 18cm between columns
            else driveDistance = 6 + (18.5 * (2 - targetColumn));
            if (driveDistance<7) driveDistance=7; // ensure turn not hitting balance stone
            StraightCm(direction*power, driveDistance); // drive to cryptobox

            if (isBlue || robot.use_newbot_v2) {
                TurnLeftD(0.35, 90);
                alignUsingIMU(0.18, 90.0);
            } else {
                TurnRightD(0.35, 90);
                alignUsingIMU(0.18, -90.0);
            }
            if (!opModeIsActive()) return;
            enable_bump_detection();
            for(int i = 0 ; i < 3 ; i++) {
                dist = Math.max(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 15;
                if (dist > 30) dist = 7;
                if (dist<1) break;
                StraightCm(-.3, dist); // drive close to cryptobox
            }
            if (robot.bump_detected) {
                // hit divider, forward 2 cm
                robot.bump_detected=false;
                StraightCm(.3, 2);
            }
            disable_bump_detection();
            if (isBlue || robot.use_newbot_v2) {
                alignUsingIMU(0.18, 90.0);
            } else {
                alignUsingIMU(0.18, -90.0);
            }
        } else { // Front box
            if (isBlue) { // Front Blue
                driveDistance = 0 + (18 * targetColumn); // 18cm between columns
            } else { // Front Red
                driveDistance = 7 + (18.5 * (2 - targetColumn)); // 18.5 cm between columns
            }
            if (!opModeIsActive()) return;

            change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
            sleep(200);
            if (isBlue)
                StraightCm(-power, driveDistance);
            else
                StraightCm(power, driveDistance);
            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
            enable_bump_detection();
            for (int i=0; i<2; i++) {
                dist = Math.max(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 15;
                if (dist<2) break;
                if (dist > 50 || (dist <= 5 && i==0)) {
                    StraightCm(-.25, 16);
                } else if (dist > 1) {
                    StraightCm(-.25, dist); // forward using front range sensor, so it is close to cryptobox
                }
            }
            if (robot.bump_detected) {
                // hit divider, forward 2 cm
                robot.bump_detected=false;
                StraightCm(.3, 2);
            }
            disable_bump_detection();
            alignUsingIMU(0.18, (isBlue?0:178));
        }
        
        change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
        // sleep(200);
        if (!opModeIsActive()) return;
        reset_prox();
        //if (nearest_col) {
        //    // allow more time for proximity sensor to reset
        //    sleep(400);
        //}
        if (isBlue) {
            driveTT(0.15, 0.15); // Crabs to the left
        } else { // Red
            driveTT(-0.15, -0.15); // Crabs to the right
        }
        if (!opModeIsActive()) return;

        boolean edge_undetected_L=true;// robot.proxSensor.getState(); // false = something within proximity
        boolean edge_undetected_R=true;
        robot.runtime.reset();
        do {
            edge_undetected_L = robot.proxL.getState();
            edge_undetected_R = (nearest_col&&isBlue?true:robot.proxR.getState());
            if (!opModeIsActive()) return;
        } while ((edge_undetected_L && edge_undetected_R) && (robot.runtime.seconds() < 1.5));

        driveTT(0, 0); // Stops

        if (!isBlue) { // crab back 3cm to correct proximity over shoot
            StraightCm(-.4, 2);
        } else {
            StraightCm(.4, 2);
        }

        if (!opModeIsActive()) return;

        change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
        sleep(300);
        StraightCm(.4, 3); // goes back so that delivery has enough space
        if (robot.use_verbose)
            telemetry.addData("0: End go_to_crypto() CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
    }

    void turnToColumn (int targetColumn, double power, boolean isBlueSide, boolean isSideBox) throws InterruptedException{
        if(isSideBox) {//Values are currently place holders, after every turn value add a forward for a fixed distance to deliver the glyph
            if (isBlueSide) {
                if (targetColumn == 0) {
                    TurnLeftD(power, 72);
                } else if (targetColumn == 1) {
                    TurnLeftD(power, 58);
                } else if (targetColumn == 2) {
                    TurnLeftD(power, 44);
                }
            } else {
                if (targetColumn == 0) {
                    TurnRightD(power, 75);
                } else if (targetColumn == 1) {
                    TurnRightD(power, 45);
                } else if (targetColumn == 2) {
                    TurnRightD(power, 30);
                }
            }
        }
        else{//Values are currently place holders
            if(isBlueSide) {
                if (targetColumn == 0) {
                    TurnLeftD(power, 75);
                } else if (targetColumn == 1) {
                    TurnLeftD(power, 45);
                } else if (targetColumn == 2) {
                    TurnLeftD(power, 30);
                }
            }
            else{
                if (targetColumn == 0) {
                    TurnRightD(power, 75);
                } else if (targetColumn == 1) {
                    TurnRightD(power, 45);
                } else if (targetColumn == 2) {
                    TurnRightD(power, 30);
                }
            }
        }
    }


    // void calc_snake(float stick_x){
    void calc_snake(float left_t, float right_t){
        float stick_x;
        if (left_t > 0.1)
            stick_x = -1 * left_t;
        else stick_x = right_t;
        if(stick_x > 0.1){
            robot.isSnakingLeft = false;
        }
        else{
            robot.isSnakingLeft = true;
        }

        if(Math.abs(stick_x) < 0.2){
            robot.enoughToSnake = false;
        }
        else{
            robot.enoughToSnake = true;
        }

        if(Math.abs(stick_x) > 0.8){
            robot.r_Value = robot.MAX_TURNING_RADIUS - (/*Hypothetical limiter here*/(robot.MAX_TURNING_RADIUS - robot.MIN_TURNING_RADIUS));
        }
        else{
            robot.r_Value = robot.MAX_TURNING_RADIUS - (Math.abs(stick_x) * (robot.MAX_TURNING_RADIUS - robot.MIN_TURNING_RADIUS));
        }

        if(robot.use_newbot){
            robot.thetaOneCalc = (Math.atan((0.5 * robot.NB_WIDTH_BETWEEN_WHEELS) / ((robot.r_Value) - (0.5 * robot.NB_LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 1 (inside wheels)
            robot.thetaTwoCalc = (Math.atan((0.5 * robot.NB_WIDTH_BETWEEN_WHEELS) / ((robot.r_Value) + (0.5 * robot.NB_LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 2 (outside wheels)
        }
        else {
            robot.thetaOneCalc = (Math.atan((0.5 * robot.WIDTH_BETWEEN_WHEELS) / ((robot.r_Value) - (0.5 * robot.LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 1
            robot.thetaTwoCalc = (Math.atan((0.5 * robot.WIDTH_BETWEEN_WHEELS) / ((robot.r_Value) + (0.5 * robot.LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 2
        }
    }

    void collectAuto() throws InterruptedException{
        StraightCm(0.5, 30);
        intakeIn();
        StraightCm(0.3, 35);
        sleep(1000);
        StraightCm(-0.3, 20);


    }

    void alignUsingIMU(double power, double tar_degree) throws InterruptedException {
        // target degree must between -180 and +180
        // align up to 45 degree
        double offset = 0;
        double cur_adj_heading = imu_heading();
        if (tar_degree>135 && cur_adj_heading<0) {
            cur_adj_heading += 360;
        } else if (tar_degree<-135 && cur_adj_heading>0) {
            cur_adj_heading -= 360;
        }

        double imu_diff = cur_adj_heading-tar_degree;
        double corrected_degree = Math.abs(imu_diff)-0.5;
        if (corrected_degree>10) {
            corrected_degree*=0.8;
        }
        if (corrected_degree>45)
            corrected_degree = 45;
        if (imu_diff < -2) {
            TurnLeftD(power, corrected_degree);
        }
        else if (imu_diff > 2) {
            TurnRightD(power, corrected_degree);
        }
        change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
    }

    void snake_servo_adj(){
        if(robot.enoughToSnake) {
            if(robot.isSnakingLeft) {
                robot.leftServoAngle = 1 - (robot.thetaOneCalc);
                robot.rightServoAngle = 1 - (robot.thetaTwoCalc);
            }
            else{
                robot.leftServoAngle = robot.thetaTwoCalc;
                robot.rightServoAngle = robot.thetaOneCalc;
            }
            robot.servoPosFL = 1 - (robot.leftServoAngle);
            robot.servoPosFR = 1 - (robot.rightServoAngle);
            robot.servoPosBL = robot.leftServoAngle;
            robot.servoPosBR = robot.rightServoAngle;
            robot.servoFrontLeft.setPosition(robot.servoPosFL);
            robot.servoFrontRight.setPosition(robot.servoPosFR);
            robot.servoBackLeft.setPosition(robot.servoPosBL);
            robot.servoBackRight.setPosition(robot.servoPosBR);
        }
        else {
            robot.set_chassis_forward_position();
        }
    }

    void car_servo_adj(double joy_stick_x){
        double degree = joy_stick_x * 25.0 / 180.0; // maximum 25 degree each way
        if(Math.abs(degree)>0.01) {

            robot.servoPosFL = robot.SERVO_FL_FORWARD_POSITION - degree;
            robot.servoPosFR = robot.SERVO_FR_FORWARD_POSITION - degree;
            robot.servoFrontLeft.setPosition(robot.servoPosFL);
            robot.servoFrontRight.setPosition(robot.servoPosFR);
        }
        else {
            robot.set_chassis_forward_position();
        }
    }

    void test_swerve_motor(double power, boolean testingForward){
        if(testingForward){
            if (robot.isTestingFL) {
                robot.motorFrontLeft.setPower(power);
            } else if (robot.isTestingFR) {
                robot.motorFrontRight.setPower(power);
            } else if (robot.isTestingBL) {
                robot.motorBackLeft.setPower(power);
            } else if (robot.isTestingBR) {
                robot.motorBackRight.setPower(power);
            }
        }
        else{
            if (robot.isTestingFL) {
                robot.motorFrontLeft.setPower(-power);
            } else if (robot.isTestingFR) {
                robot.motorFrontRight.setPower(-power);
            } else if (robot.isTestingBL) {
                robot.motorBackLeft.setPower(-power);
            } else if (robot.isTestingBR) {
                robot.motorBackRight.setPower(-power);
            }
        }

    }

    void test_swerve_servo(boolean isTestingLeft){
        if(isTestingLeft){
            if (robot.isTestingFL) {
                robot.servoFrontLeft.setPosition(robot.servoPosFL + 0.01);
                robot.servoPosFL += 0.01;
            } else if (robot.isTestingFR) {
                robot.servoFrontRight.setPosition(robot.servoPosFR + 0.01);
                robot.servoPosFR += 0.01;
            } else if (robot.isTestingBL) {
                robot.servoBackLeft.setPosition(robot.servoPosBL + 0.01);
                robot.servoPosBL += 0.01;
            } else if (robot.isTestingBR) {
                robot.servoBackRight.setPosition(robot.servoPosBR + 0.01);
                robot.servoPosBR += 0.01;
            } else {

                robot.servoFrontLeft.setPosition(robot.servoPosFL + 0.05);
                robot.servoPosFL += 0.05;

                robot.servoFrontRight.setPosition(robot.servoPosFR + 0.05);
                robot.servoPosFR += 0.05;

                robot.servoBackLeft.setPosition(robot.servoPosBL + 0.05);
                robot.servoPosBL += 0.05;

                robot.servoBackRight.setPosition(robot.servoPosBR + 0.05);
                robot.servoPosBR += 0.05;
            }
        }
        else{
            if (robot.isTestingFL) {
                robot.servoFrontLeft.setPosition(robot.servoPosFL - 0.01);
                robot.servoPosFL -= 0.01;
            } else if (robot.isTestingFR) {
                robot.servoFrontRight.setPosition(robot.servoPosFR - 0.01);
                robot.servoPosFR -= 0.01;
            } else if (robot.isTestingBL) {
                robot.servoBackLeft.setPosition(robot.servoPosBL - 0.01);
                robot.servoPosBL -= 0.01;
            } else if (robot.isTestingBR) {
                robot.servoBackRight.setPosition(robot.servoPosBR - 0.01);
                robot.servoPosBR -= 0.01;
            } else {

                robot.servoFrontLeft.setPosition(robot.servoPosFL - 0.05);
                robot.servoPosFL -= 0.05;

                robot.servoFrontRight.setPosition(robot.servoPosFR - 0.05);
                robot.servoPosFR -= 0.05;

                robot.servoBackLeft.setPosition(robot.servoPosBL - 0.05);
                robot.servoPosBL -= 0.05;

                robot.servoBackRight.setPosition(robot.servoPosBR - 0.05);
                robot.servoPosBR -= 0.05;
            }
        }
    }

    void correct_swerve_servos(){
        // Normalize the values so neither exceed 1.0 or 0.0

        if (Math.abs(robot.motorPowerLeft) > 1) {
            robot.motorPowerLeft = 1.0;
        }
        if (Math.abs(robot.motorPowerRight) > 1) {
            robot.motorPowerRight = 1.0;
        }
        if (Math.abs(robot.servoPosFL) > 1) {
            robot.servoPosFL = 1.0;
        }
        if (Math.abs(robot.servoPosFR) > 1) {
            robot.servoPosFR = 1.0;
        }
        if (Math.abs(robot.servoPosBL) > 1) {
            robot.servoPosBL = 1.0;
        }
        if (Math.abs(robot.servoPosBR) > 1) {
            robot.servoPosBR = 1.0;
        }
        if (Math.abs(robot.servoPosFL) < 0) {
            robot.servoPosFL = 0.0;
        }
        if (Math.abs(robot.servoPosFR) < 0) {
            robot.servoPosFR = 0.0;
        }
        if (Math.abs(robot.servoPosBL) < 0) {
            robot.servoPosBL = 0.0;
        }
        if (Math.abs(robot.servoPosBR) < 0) {
            robot.servoPosBR = 0.0;
        }
    }

    void change_swerve_pos(SwerveDriveHardware.CarMode new_mode) {
        if (new_mode == SwerveDriveHardware.CarMode.TURN) {
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_TURN_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_TURN_POSITION);
            robot.servoBackLeft.setPosition(robot.SERVO_BL_TURN_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_TURN_POSITION);

            robot.servoPosFL = robot.SERVO_FL_TURN_POSITION;
            robot.servoPosFR = robot.SERVO_FR_TURN_POSITION;
            robot.servoPosBL = robot.SERVO_BL_TURN_POSITION;
            robot.servoPosBR = robot.SERVO_BR_TURN_POSITION;
        }
        else if(new_mode == SwerveDriveHardware.CarMode.CRAB){
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_STRAFE_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_STRAFE_POSITION);
            robot.servoBackLeft.setPosition(robot.SERVO_BL_STRAFE_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_STRAFE_POSITION);

            robot.servoPosFL = robot.SERVO_FL_STRAFE_POSITION;
            robot.servoPosFR = robot.SERVO_FR_STRAFE_POSITION;
            robot.servoPosBL = robot.SERVO_BL_STRAFE_POSITION;
            robot.servoPosBR = robot.SERVO_BR_STRAFE_POSITION;
        }
        else if(new_mode == SwerveDriveHardware.CarMode.STRAIGHT || new_mode == SwerveDriveHardware.CarMode.CAR){
            robot.set_chassis_forward_position();
        }
        else if(new_mode == SwerveDriveHardware.CarMode.ORBIT){
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_ORBIT_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_ORBIT_POSITION);
            robot.servoBackLeft.setPosition(robot.SERVO_BL_ORBIT_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_ORBIT_POSITION);

            robot.servoPosFL = robot.SERVO_FL_ORBIT_POSITION;
            robot.servoPosFR = robot.SERVO_FR_ORBIT_POSITION;
            robot.servoPosBL = robot.SERVO_BL_ORBIT_POSITION;
            robot.servoPosBR = robot.SERVO_BR_ORBIT_POSITION;
        }
        robot.old_mode = robot.cur_mode;
        robot.cur_mode = new_mode;

        robot.isTestingFL = false;
        robot.isTestingFR = false;
        robot.isTestingBL = false;
        robot.isTestingBR = false;
    }

    // void set_swerve_power(float right_stick, float left_stick, float x_stick){

    void toggleDriveSpeed(boolean isToggled){
        if(isToggled){
            robot.drivePowerRatio = 0.2;
        }
        else{
            robot.drivePowerRatio = 0.5;
        }
    }

    void set_swerve_power(float right_stick, float left_stick, float left_t, float right_t, boolean isK){
        float x_stick = 0;
        if (left_t > 0.1)
            x_stick = -1 * left_t;
        else x_stick = right_t;

        if(robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
            if(robot.use_newbot){
                robot.insideWheelsMod = left_stick * ((Math.pow((Math.pow(0.5 * robot.NB_LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) - robot.NB_WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                        (robot.r_Value));
                robot.outsideWheelsMod = left_stick * ((Math.pow((Math.pow(0.5 * robot.NB_LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) + robot.NB_WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                        (robot.r_Value));
            }
            else {
                robot.insideWheelsMod = left_stick * ((Math.pow((Math.pow(0.5 * robot.LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) - robot.WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                        (robot.r_Value));
                robot.outsideWheelsMod = left_stick * ((Math.pow((Math.pow(0.5 * robot.LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) + robot.WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                        (robot.r_Value));
            }
            if (robot.enoughToSnake) {
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                if (robot.isSnakingLeft) {
                    robot.motorPowerLeft = robot.insideWheelsMod;
                    robot.motorPowerRight = robot.outsideWheelsMod;
                } else {
                    robot.motorPowerLeft = robot.outsideWheelsMod;
                    robot.motorPowerRight = robot.insideWheelsMod;
                }
            } else {
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                robot.motorPowerLeft = left_stick;
                robot.motorPowerRight = left_stick;
            }

            if(isK) {

                if (robot.enoughToSnake) {
                    robot.motorFrontLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                    robot.motorFrontRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
                    if (!robot.use_front_drive_only) {
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }
                } else {
                    robot.motorFrontLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                    robot.motorFrontRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
                    if (!robot.use_front_drive_only) {
                        robot.motorBackLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                        robot.motorBackRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
                    }
                }
            }
            else{
                if (robot.enoughToSnake) {
                    robot.motorFrontLeft.setPower(robot.motorPowerLeft);
                    robot.motorFrontRight.setPower(robot.motorPowerRight);
                    if (!robot.use_front_drive_only) {
                        robot.motorBackLeft.setPower(robot.motorPowerLeft);
                        robot.motorBackRight.setPower(robot.motorPowerRight);
                    }
                    else{
                        robot.motorBackLeft.setPower(0);
                        robot.motorBackRight.setPower(0);
                    }
                } else {
                    robot.motorFrontLeft.setPower(robot.motorPowerLeft);
                    robot.motorFrontRight.setPower(robot.motorPowerRight);
                    if (!robot.use_front_drive_only) {
                        robot.motorBackLeft.setPower(robot.motorPowerLeft);
                        robot.motorBackRight.setPower(robot.motorPowerRight);
                    }
                }
            }

        } else if (robot.cur_mode == SwerveDriveHardware.CarMode.TURN) {
            robot.motorPowerTurn = x_stick;
            robot.motorFrontLeft.setPower(-robot.motorPowerTurn * robot.drivePowerRatio);
            robot.motorFrontRight.setPower(robot.motorPowerTurn * robot.drivePowerRatio);
            if (!robot.use_front_drive_only) {
                robot.motorBackLeft.setPower(-robot.motorPowerTurn * robot.drivePowerRatio);
                robot.motorBackRight.setPower(robot.motorPowerTurn * robot.drivePowerRatio);
            }
        } else {
            robot.motorPowerLeft = left_stick;
            robot.motorPowerRight = right_stick;
            if (robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT) {
                robot.motorFrontLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                robot.motorFrontRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                    robot.motorBackRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
                }
            } else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB) {
                robot.motorFrontLeft.setPower(-robot.motorPowerRight * robot.drivePowerRatio);
                robot.motorFrontRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
                if (!robot.use_front_drive_only) {
                    robot.motorBackLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                    robot.motorBackRight.setPower(-robot.motorPowerLeft * robot.drivePowerRatio);
                }
            }
        }
    }

    static class Camera {
        private VuforiaLocalizer vuforia;
        private String lastError = "";

        Camera(VuforiaLocalizer vuforia){
            this.vuforia = vuforia;
        }

        void activate() {
            this.vuforia.setFrameQueueCapacity(1);
            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        }

        String getLastError() {
            return lastError;
        }

        private Bitmap convertFrameToBitmap(VuforiaLocalizer.CloseableFrame frame) {
            long numImages = frame.getNumImages();
            Image image = null;

            for (int imageI = 0; imageI < numImages; imageI++) {
                if (frame.getImage(imageI).getFormat() == PIXEL_FORMAT.RGB565) {
                    image = frame.getImage(imageI);
                    break;
                }
            }
            if (image == null) {
                for (int imageI = 0; imageI < numImages; imageI++) {
                    //For diagnostic purposes
                }
                lastError = "Failed to get RGB565 image format!";
                return null;
            }
            Bitmap bitmap = Bitmap.createBitmap(image.getWidth(),image.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(image.getPixels());
            return bitmap;
        }

        private Bitmap cropBitmap(Bitmap source, double xOffsetF, double yOffsetF, double widthF, double heightF) {
            int offset_x = (int)(source.getWidth() * xOffsetF);
            int offset_y = (int)(source.getHeight() * yOffsetF);
            int width = (int)(source.getWidth() * widthF);
            int height = (int)(source.getHeight() * heightF);
            Bitmap destBitmap = Bitmap.createBitmap(source, offset_x, offset_y, width, height);
            return destBitmap;
        }

        /**
         * @param xOffsetF
         * @param yOffsetF
         * @param widthF
         * @param heightF
         * @return
         */
        Bitmap captureBitmap(double xOffsetF, double yOffsetF, double widthF, double heightF) {
            Bitmap bitmapTemp = null;
            lastError = "";
            int capacity = vuforia.getFrameQueueCapacity();
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().poll();

            if (frame == null) {
                lastError = "ERROR! Failed to retrieve frame!";
                return null;
            }

            bitmapTemp = convertFrameToBitmap(frame);

            frame.close();
            if (bitmapTemp == null) {
                lastError = "ERROR! Failed to retrieve bitmap";
                return null;

            }
            //White Balance applied here
            int whitestPixel = getWhitestPixel(bitmapTemp);
            applyWhiteBalance(bitmapTemp, whitestPixel);

            //Bitmap bitmap = cropBitmap(bitmapTemp, xOffsetF, yOffsetF, widthF, heightF);
            Bitmap bitmap = bitmapTemp;
            return bitmap;
        }

        /**
         * Takes a Bitmap as input and outputs an integer of the color constant of the "whitest" pixel
         * @param source
         * @return
         */
        int getWhitestPixel(Bitmap source){
            int[] pixels = new int[source.getHeight() * source.getWidth()];

            int whitestPixel = 0;
            int whitestPixelColorAvg = 0;

            source.getPixels(pixels, 0, source.getWidth(), 0, 0, source.getWidth(), source.getHeight());

            for(int pixeli = 0; pixeli < pixels.length; pixeli++){

                int pixelRed = Color.red(pixels[pixeli]);
                int pixelGreen = Color.green(pixels[pixeli]);
                int pixelBlue = Color.blue(pixels[pixeli]);

                int pixeliColorAvg = (pixelRed + pixelGreen + pixelBlue) / 3;

                if(pixeliColorAvg > whitestPixelColorAvg){
                    whitestPixel = pixeliColorAvg;
                    whitestPixelColorAvg = pixeliColorAvg;
                }
            }
            if (whitestPixel / Color.WHITE > .8){
                return whitestPixel;
            }
            else{
                whitestPixel = 2;
                return whitestPixel;
            }
        }

        /**
         * Takes returned integer from getWhitestPixel and a source bitmap then returns a white balanced bitmap
         * @param source
         * @param whitestPixel
         * @return
         */
        Bitmap applyWhiteBalance(Bitmap source, int whitestPixel) {
            if (whitestPixel == 2){
                return source;
            }
            else {
                if (Color.red(whitestPixel) != 0 && Color.green(whitestPixel) != 0 && Color.red(whitestPixel) != 0) {
                    double rComp = 255.0 / Color.red(whitestPixel);
                    double gComp = 255.0 / Color.green(whitestPixel);
                    double bComp = 255.0 / Color.blue(whitestPixel);

                    int w = source.getWidth();
                    int h = source.getHeight();

                    for (int i = 0; i < w; i++) {
                        for (int j = 0; j < h; j++) {
                            int pixColor = source.getPixel(i, j);

                            int inR = Color.red(pixColor);
                            int inG = Color.green(pixColor);
                            int inB = Color.blue(pixColor);

                            double rDoub = inR * rComp;
                            double gDoub = inG * gComp;
                            double bDoub = inB * bComp;

                            source.setPixel(i, j, Color.argb(255, (int) rDoub, (int) gDoub, (int) bDoub));

                            if (source.getConfig() != Bitmap.Config.RGB_565) {
                                source.setConfig(Bitmap.Config.RGB_565);
                            }
                        }
                    }
                }
                return source;
            }
        }
        void stopCamera(){
            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, false);
            this.vuforia.setFrameQueueCapacity(0);
        }
    }

//      Needs refinement of Array logic to get true gray int (-7829368)
//        int getGrayestPixel(Bitmap source){
//            int[] pixels = new int[source.getHeight() * source.getWidth()];
//
//            int grayestPixel = 0;
//
//            source.getPixels(pixels, 0, 1, 0, 0, source.getWidth(), source.getHeight());
//
//            Arrays.sort(pixels);
//
//            pixels[pixels.length / 2] = grayestPixel;
//            return grayestPixel;
//        }






    /**
     * Determines if jewel color is blue, red, or other/unsure.
     * Expects majority of bitmap to be either red or blue.
     * Outputs string as either "Red", "Blue", or "Unsure"
     * @param bitmap
     * @return String
     */
    TeamColor determineJewelColor(Bitmap bitmap) {
        int height = bitmap.getHeight();
        int width = bitmap.getWidth();

        int[] pixels = new int[bitmap.getWidth()];

        if (robot.allianceColor == TeamColor.BLUE){
            bitmap.getPixels(pixels, 0, bitmap.getWidth(), 0, bitmap.getHeight() * 7 / 8, bitmap.getWidth(), 1);
        }
        else if (robot.allianceColor == TeamColor.RED){
            bitmap.getPixels(pixels, 0, bitmap.getWidth(), 0, bitmap.getHeight() * 5 / 6, bitmap.getWidth(), 1);
        }
        else if (robot.allianceColor == TeamColor.UNKNOWN){
            throw new IllegalStateException("Variable allianceColor is set to Unknown");
        } else {
            throw new IllegalStateException("There is something wrong with variable allianceColor");
        }
//        int redTotal = 0;
//        int blueTotal = 0;
//        int redValue = 0;
//        int blueValue = 0;

//        int pixelTR = bitmap.getPixel(bitmap.getWidth() / 2, bitmap.getHeight() / 2);
//                int pixelBR = bitmap.getPixel(bitmap.getWidth() * 3 / 4,bitmap.getHeight() / 4);
//                int pixelTL = bitmap.getPixel(bitmap.getWidth() /4, bitmap.getHeight() - 1);
//                int pixelBL = bitmap.getPixel(bitmap.getWidth() / 4,bitmap.getHeight() / 4);
//                int pixelM = bitmap.getPixel(bitmap.getWidth() / 2,bitmap.getHeight() / 2);

//        telemetry.addData("Pixel", String.format("%x", pixelTR));
//                telemetry.addData("PixelBR", String.format("%x", pixelBR));
//                telemetry.addData("PixelTL", String.format("%x", pixelTL));
//                telemetry.addData("PixelBL", String.format("%x", pixelBL));
//                telemetry.addData("PixelM", String.format("%x", pixelM));

        int redCount = 0;
        int blueCount = 0;
        int unknownCount = 0;

        int redStart = -1;
        int blueStart = -1;

        for (int pixelI = 0; pixelI < pixels.length; pixelI++){
            int pixelRed = Color.red(pixels[pixelI]);
            int pixelGreen = Color.green(pixels[pixelI]);
            int pixelBlue = Color.blue(pixels[pixelI]);

            double greenToBlueRatio = (double)pixelGreen / (double)pixelBlue;
            double greenToRedRatio = (double)pixelGreen / (double)pixelRed;

            if((pixelRed > (pixelGreen * 1.4)) && (pixelRed > (pixelBlue * 1.4)) && (greenToRedRatio < .3)){
                redCount++;
                if (redStart == -1){
                    redStart = pixelI;
                }
            }
            else if ((pixelBlue > (pixelRed * 1.4)) && (greenToBlueRatio > .65) && (greenToBlueRatio < .9)){
                blueCount++;
                if (blueStart == -1){
                    blueStart = pixelI;
                }
            }
            else {
                unknownCount++;
            }
        }

        telemetry.addData("Red Count", redCount).setRetained(true);
        telemetry.addData("Blue Count", blueCount).setRetained(true);
        telemetry.addData("Unknown Count", unknownCount).setRetained(true);
        telemetry.addData("Red Start", redStart).setRetained(true);
        telemetry.addData("Blue Start", blueStart).setRetained(true);

        if(redStart < blueStart && redStart != -1 && blueStart != -1){
            return TeamColor.RED;
        }
        else if (blueStart < redStart && blueStart != -1 && redStart != -1){
            return TeamColor.BLUE;
        } else {
            return TeamColor.UNKNOWN;
        }

//        for (int pixelI = 0; pixelI < pixels.length; pixelI++) {
//            int b = Color.blue(pixels[pixelI]);
//            int r = Color.red(pixels[pixelI]);
//
//            if (r > b) {
//                redTotal++;
//                redValue += r;
//            }
//            else {
//                blueTotal++;
//                blueValue += b;
//            }
//        }
//        if (redTotal > 1.3 * blueTotal) {
//            return TeamColor.RED;
//
//        }
//        else if (blueTotal > 1.3 * redTotal){
//            return TeamColor.BLUE;
//
//        }
//        else {
//            return TeamColor.UNKNOWN;
//        }
//        telemetry.addData("Red Total", redTotal);
//        telemetry.addData("Blue Total", blueTotal);
//        telemetry.addData("Red Value AVG", redTotal > 0 ? redValue/redTotal : 0);
//        telemetry.addData("Blue Value AVG", blueTotal > 0 ? blueValue/blueTotal : 0);
    }

    void dumperGateUp() {
        if (!robot.use_dumper || !robot.use_newbot_v2)
            return;
        robot.sv_dumper_gate.setPosition(robot.SV_DUMPER_GATE_UP);
    }

    void dumperGateDown() {
        if (!robot.use_dumper || !robot.use_newbot_v2)
            return;
        robot.sv_dumper_gate.setPosition(robot.SV_DUMPER_GATE_DOWN);
    }

    void intakeGateInit() {
        if (!robot.use_intake || !robot.use_newbot_v2)
            return;
        robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_INIT);
    }

    void intakeGateUp() {
        if (!robot.use_intake || !robot.use_newbot_v2)
            return;
        double pos = robot.sv_intake_gate.getPosition();
        if (Math.abs(pos-robot.SV_INTAKE_GATE_DOWN)<0.1)
            robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_MID);
        else if (Math.abs(pos-robot.SV_INTAKE_GATE_MID)<0.1)
            robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_UP);
        else
            robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_INIT);
    }

    void intakeGateMid() {
        if (!robot.use_intake || !robot.use_newbot_v2)
            return;
        robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_MID);
    }


    void intakeGateDown() {
        if (!robot.use_intake || !robot.use_newbot_v2)
            return;
        if (robot.use_newbot_v2 && robot.use_relic_grabber)
            relic_arm_up();
        double pos = robot.sv_intake_gate.getPosition();
        if (Math.abs(pos-robot.SV_INTAKE_GATE_INIT)<0.1)
            robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_UP);
        else if (Math.abs(pos-robot.SV_INTAKE_GATE_UP)<0.1)
            robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_MID);
        else
            robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_DOWN);
    }

    void intakeBarWheelIn() {
        if (robot.sv_bar_wheel==null)
            return;
        robot.sv_bar_wheel.setPower(0.8);
    }

    void intakeBarWheelOut() {
        if (robot.sv_bar_wheel==null)
            return;
        robot.sv_bar_wheel.setPower(-0.8);
    }

    void intakeBarWheelInP(double pw)
    {
        if (robot.sv_bar_wheel==null)
            return;
        robot.sv_bar_wheel.setPower(Math.abs(pw));
    }

    void intakeBarWheelOutP(double pw) {
        if (robot.sv_bar_wheel==null)
            return;
        robot.sv_bar_wheel.setPower(-1*Math.abs(pw));
    }

    void intakeBarWheelStop() {
        if (robot.sv_bar_wheel==null)
            return;
        robot.sv_bar_wheel.setPower(0);
    }

    void intakeIn() {
        if (!robot.use_intake)
            return;
        if (robot.sv_dumper!=null && robot.sv_dumper.getPosition()<0.63) {
            return;
        }
        robot.mt_intake_left.setPower(robot.intakeRatio);
        robot.mt_intake_right.setPower(robot.intakeRatio);
        intakeBarWheelIn();
    }

    void correctGlyph(boolean leadClockwise) {
        if (!robot.use_intake)
            return;
        if(leadClockwise) {
            intakeTurn(true);
            sleep(150);
            intakeTurn(false);
            sleep(150);
            intakeIn();
            sleep(300);
        }
        else{
            intakeTurn(false);
            sleep(150);
            intakeTurn(true);
            sleep(150);
            intakeIn();
            sleep(300);
        }
    }

    void intakeTurn(boolean clockwise) {
        if (!robot.use_intake)
            return;
        if (clockwise) {
            robot.mt_intake_left.setPower(-robot.intakeRatio / 2.0);
            robot.mt_intake_right.setPower(robot.intakeRatio);
        } else {
            robot.mt_intake_left.setPower(robot.intakeRatio);
            robot.mt_intake_right.setPower(-robot.intakeRatio / 2);
        }
    }

    void intakeOut() {
        if (!robot.use_intake)
            return;
        robot.mt_intake_left.setPower(-1.0*robot.intakeRatio);
        robot.mt_intake_right.setPower(-1.0*robot.intakeRatio);
        intakeBarWheelOut();
    }

    void intakeStop() {
        if (!robot.use_intake)
            return;
        robot.mt_intake_left.setPower(0);
        robot.mt_intake_right.setPower(0);
        intakeBarWheelStop();
    }

    void show_telemetry() throws InterruptedException {
        telemetry.addData("1. Team", " %s sw/IMU/Vu/GG = %s%s/%s/%s/%s",
                robot.allianceColor, (robot.use_swerve||robot.use_newbot ?"Y":"N"),
                (robot.isTesting?"-T":""), (robot.use_imu?"Y":"N"),
                (robot.use_Vuforia ?"Y":"N"), (robot.use_glyph_grabber ?"Y":"N"));
        telemetry.addData("2. PW-r/L/R/Rot-En/Sl-En =", "%.2f,%.2f/%.2f/%s/%s",
                robot.drivePowerRatio, robot.motorPowerLeft,robot.motorPowerRight,
                (robot.gg_rotator_encoder_ok ?"Y":"N"),(robot.gg_slider_encoder_ok ?"Y":"N"));
        telemetry.addData("3. W-sv angle FL/FR/BL/BR =", "%.3f/%.3f/%.3f/%.3f",
                robot.servoPosFL, robot.servoPosFR, robot.servoPosBL, robot.servoPosBR);
        double range_front_left = getRange(RangeSensor.FRONT_LEFT);
        double range_front_right = getRange(RangeSensor.FRONT_RIGHT);
        double range_back = getRange(RangeSensor.BACK);
        boolean glyph_stuck = GlyphStuck();
        if (robot.use_imu||robot.use_range_sensor) {
            telemetry.addData("4.1 IMU/Range", "%s=%.2f(i2=%.0f)/L=%.0f/R=%.0f/B%s=%.0fcm",
                    (robot.use_imu2?"i2":"i1"),imu_heading(),imu2_heading(), range_front_left,range_front_right,
                    (glyph_stuck?"(S)":""), range_back);
        }
        if (robot.use_proximity_sensor) {
            if (robot.use_newbot) {
                telemetry.addData("4.2.1 ProxSensorLeft =", robot.proxL.getState());
                telemetry.addData("4.2.2 ProxSensorRight =", robot.proxR.getState());
            }
            else {
                telemetry.addData("4.2 ProxSensor =", robot.proxL.getState());
            }
        }
        if (robot.use_color_sensor) {
            if (robot.use_newbot_v2) {
                telemetry.addData("5.2 color =", "L b/r = %3d/%3d, R b/r = %3d/%3d",
                        robot.l_colorSensor.blue(), robot.l_colorSensor.red(),
                        robot.r_colorSensor.blue(), robot.r_colorSensor.red());
            } else {
                telemetry.addData("5.2 color =", "R b/r = %3d/%3d",
                        robot.r_colorSensor.blue(), robot.r_colorSensor.red());
            }
        }

        if (robot.use_Vuforia) {
            telemetry.addData("5. Vuforia Column = ", "%d", get_cryptobox_column());
        }
        if ((robot.use_swerve || robot.use_newbot)&& !robot.servo_tune_up) {
            telemetry.addData("6. Drive Mode = ", "%s", robot.cur_mode);
            telemetry.addData("6.1 SW-Enc FL/FR/BL/BR = ", "%d/%d/%d/%d",
                    robot.motorFrontLeft.getCurrentPosition(),
                    robot.motorFrontRight.getCurrentPosition(),
                    (robot.motorBackLeft!=null?robot.motorBackLeft.getCurrentPosition():0),
                    (robot.motorBackRight!=null?robot.motorBackRight.getCurrentPosition():0));
        }
        if (robot.use_intake) {
            telemetry.addData("7. Intake r = ", "%2.2f", robot.intakeRatio);
        }

        if (robot.use_dumper)  {
            telemetry.addData("8.1 dumper slide / pos = ","%d (pw=%.2f)/%3.3f",
                    robot.mt_lift.getCurrentPosition(), robot.mt_lift.getPower(),
                    robot.sv_dumper.getPosition());
        }

        if (robot.use_glyph_grabber)  {
            telemetry.addData("8.1 g-rot pw/cur/tar = ","%3.2f/%d/%d (%s)",
                    robot.mt_glyph_rotator.getPower(),robot.mt_glyph_rotator.getCurrentPosition(),
                    robot.target_rot_pos, (robot.is_gg_upside_down ?"dw":"up"));
            telemetry.addData("8.2 g-slider pw/cur/tar = ","%3.2f/%d/%d (%d)",
                    robot.mt_glyph_slider.getPower(),robot.mt_glyph_slider.getCurrentPosition(),
                    robot.target_gg_slider_pos, robot.gg_layer);
            telemetry.addData("8.3 gg top/bottom = ","%4.3f/%4.3f (%s)",
                    robot.sv_glyph_grabber_top.getPosition(),
                    robot.sv_glyph_grabber_bottom.getPosition(),
                    (robot.is_gg_upside_down ?"dw":"up"));
        } else if (robot.use_test_motor) {
            telemetry.addData("8. gg-rot pw/cur/tar = ","%3.2f/%d/%d(%s)",
                    robot.mt_test.getPower(),robot.mt_test.getCurrentPosition(),robot.target_rot_pos,
                    (robot.is_gg_upside_down ?"dw":"up"));
        }
        if (robot.use_test_servo) {
            // telemetry.addData("10. test sv = ","%.3f",robot.sv_test.getPosition());
        }
        if (robot.use_relic_grabber) {
            telemetry.addData("9.1 relic gr/wrist/elbow = ", "%4.4f/%4.3f/%4.3f",
                    robot.sv_relic_grabber.getPosition(), robot.sv_relic_wrist.getPosition(),
                    (robot.sv_relic_elbow!=null?robot.sv_relic_elbow.getPosition():0));
        }
        if (robot.use_relic_slider) {
            telemetry.addData("9.2 r-slider pwr/enc/tar = ","%3.2f/%d/%d",
                    robot.mt_relic_slider.getPower(),robot.mt_relic_slider.getCurrentPosition(),robot.target_relic_slider_pos);
        }
        if (robot.isTesting){
            if(robot.use_newbot){
                telemetry.addData("11. CRAB_LEFT_DIFF/CRAB_RIGHT_DIFF = ", "%.4f/%.4f", robot.NB_LEFT_SV_DIFF, robot.NB_RIGHT_SV_DIFF);
                telemetry.addData("12. CRAB_90_DIFF_DEC_FR/BR = ", "%.4f/%.4f",robot.NB_CRAB_DIFF_DEC_FR, robot.NB_CRAB_DIFF_DEC_BR);
                telemetry.addData("13. CRAB_90_DIFF_INC_FL/BL = ", "%.4f/%.4f", robot.NB_CRAB_DIFF_INC_FL, robot.NB_CRAB_DIFF_INC_BL);
            }
            else {
                telemetry.addData("11. CRAB_LEFT_DIFF/CRAB_RIGHT_DIFF = ", "%.4f/%.4f", robot.LEFT_SV_DIFF, robot.RIGHT_SV_DIFF);
            }
        }
        telemetry.update();
    }
}
