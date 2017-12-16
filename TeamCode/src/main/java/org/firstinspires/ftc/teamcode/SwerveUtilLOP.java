package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import java.util.Arrays;
import java.util.concurrent.TimeoutException;

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

    @Override
    public void runOpMode() throws InterruptedException{
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

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
        robot.init(hardwareMap);

        if (robot.use_glyph_grabber) {
            // test_glyph_rotator_encoder();
            robot.gg_rotator_encoder_ok = true;
            // test_glyph_slider_encoder();
            robot.gg_slider_encoder_ok = true;
        }
        if (robot.use_Vuforia) {
            robot.targetColumn = get_cryptobox_column();
            telemetry.addData("0: Crypto Column =", robot.targetColumn);
        }
    }

    public void start_init() {
        glyph_grabber_auto_open();
        glyph_slider_init();
    }

    public double imu_heading() {
        if (!robot.use_imu)
            return -1.0;

        robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return robot.angles.firstAngle;
    }

    public void glyph_grabber_close() {
        if (robot.is_gg_upside_down) { // close up grabber
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_CLOSED);
        } else {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_CLOSED);
        }
    }

    public void glyph_grabber_auto_close() { // close the down/up grabber depend on upside down
        if (robot.is_gg_upside_down) { // close up grabber
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_CLOSED);
            sleep(300);
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_HALF_CLOSED);
            sleep(300);
            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
            driveTT(-0.2, -0.2);
            sleep(200);
            driveTT(0,0);
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_CLOSED);
            robot.gg_top_close = true;
        } else {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_CLOSED);
            sleep(300);
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_HALF_CLOSED);
            sleep(300);
            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
            driveTT(-0.2, -0.2);
            sleep(200);
            driveTT(0,0);
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_CLOSED);
            robot.gg_bottom_close = true;

        }
    }
    public void glyph_grabber_all_close() { // close the down/up grabber depend on upside down
        robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_CLOSED);
        robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_CLOSED);
        robot.gg_top_close = true;
        robot.gg_bottom_close = true;

    }

    public void glyph_grabber_half_close() {
        if (robot.is_gg_upside_down) { // close up grabber
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_HALF_CLOSED);
            robot.gg_top_close = true;
        } else {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_HALF_CLOSED);
            robot.gg_bottom_close = true;
        }
    }

    public void glyph_grabber_half_close_both() {
        robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_HALF_CLOSED);
        robot.gg_top_close = true;
        robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_HALF_CLOSED);
        robot.gg_bottom_close = true;
    }

    public void glyph_grabber_auto_open() { // open both grabbers
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
        if (robot.is_gg_upside_down) {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_OPEN);
        } else {
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_OPEN);
        }
    }

    public void glyph_grabber_open_bottom() { // open top grabber
        if (robot.is_gg_upside_down) {
            robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_OPEN);
        } else {
            robot.sv_glyph_grabber_bottom.setPosition(robot.SV_GLYPH_GRABBER_BOTTOM_OPEN);
        }
    }

    public void glyph_grabber_open_and_push() { // open both grabbers
        glyph_grabber_auto_open();
        sleep(500);
        driveTT(-0.2,-0.2);
        sleep(500);
        driveTT(0.6,0.6);
        sleep(100);
        driveTT(0,0);
    }

    public void glyph_grabber_open_and_push_auto() { // open both grabbers
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
        // if grabber is close and at ladder 0, need to slide up a little bit before rotation
        boolean need_slide_up = false;
        stop_chassis();
        int orig_slide_pos = robot.mt_glyph_rotator.getCurrentPosition();
        if (robot.gg_layer==0 && (orig_slide_pos<800 || robot.is_gg_upside_down==true)) {
            need_slide_up = true;
        }
        if (orig_slide_pos>1000) { // more than 4.5 inches above the ground
            need_slide_up = false;
        }
        double up_inches = (1200-orig_slide_pos) / 300.0;
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
        robot.is_gg_upside_down = !robot.is_gg_upside_down;
        boolean need_slide_back = need_slide_up;
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
            glyph_slider_position(back_pos);
            sleep(500);
        }
    }

    public void rotate_refine() {
        rotate_to_target(0.2);
    }

    public void relic_grabber_close() {
       robot.sv_relic_grabber.setPosition(robot.SV_RELIC_GRABBER_CLOSE);
    }

    public void relic_grabber_open() {
        robot.sv_relic_grabber.setPosition(robot.SV_RELIC_GRABBER_OPEN);
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
        slide_to_target(.5);
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

    void glyph_slider_up() {
        robot.mt_glyph_slider_pw = 0.7;
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
        robot.mt_glyph_slider_pw = -0.4;
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

    public void slide_to_target(double power) {
        if (!robot.gg_slider_encoder_ok)
            return;

        if (power<0) power=-1.0*power;
        robot.mt_glyph_slider.setTargetPosition(robot.target_gg_slider_pos);
        robot.mt_glyph_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.runtime.reset();
        robot.mt_glyph_slider.setPower(Math.abs(power));
        while (robot.mt_glyph_slider.isBusy() && (robot.runtime.seconds()<3) && opModeIsActive()) {
            telemetry.addData("8. gg-rot pwr/cur/tar = ","%3.2f/%d/%d",
                    robot.mt_glyph_slider.getPower(),robot.mt_glyph_slider.getCurrentPosition(),robot.target_gg_slider_pos);
            telemetry.update();
        }
        robot.mt_glyph_slider.setPower(Math.abs(power/2.0));
        while (robot.mt_glyph_slider.isBusy() && (robot.runtime.seconds()<1) && opModeIsActive()) {
            telemetry.addData("8. gg-rot pwr/cur/tar = ","%3.2f/%d/%d",
                    robot.mt_glyph_slider.getPower(),robot.mt_glyph_slider.getCurrentPosition(),robot.target_gg_slider_pos);
            telemetry.update();
        }
        robot.mt_glyph_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mt_glyph_slider.setPower(0);
    }

    public void driveTT(double lp, double rp) {
        if(!robot.fast_mode && robot.straight_mode) { // expect to go straight
            if (robot.use_imu) {
                double cur_heading = imu_heading();
                double heading_off_by = ((cur_heading - robot.target_heading) / 360);
                if(robot.use_swerve) {
                    if(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                        if(rp > 0 && lp > 0) { //When going forward
                            if(true) {
                                if (cur_heading - robot.target_heading > 0.7) {
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION - heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION - heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) {
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION + heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION + heading_off_by);
                                }
                            }
                        }
                        else{ // When going backward
                            if(true) {
                                if (cur_heading - robot.target_heading > 0.7) { //Drifting to the left
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION + heading_off_by); //Turn Front servos to the right
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION + heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) { //Drifting to the right
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION - heading_off_by); //Turn Front servos to the left
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION - heading_off_by);
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
        if(robot.use_swerve) {
            if (robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                robot.motorBackLeft.setPower(lp);
                robot.motorBackRight.setPower(rp);

            } else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(-rp);
                robot.motorBackLeft.setPower(lp);
                robot.motorBackRight.setPower(-lp);
            }
            else if(robot.cur_mode == SwerveDriveHardware.CarMode.TURN) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                robot.motorBackLeft.setPower(lp);
                robot.motorBackRight.setPower(rp);
            }
        }
        else{
            if (Math.abs(rp) > 0.3 && Math.abs(lp) > 0.3) {
                robot.motorFrontRight.setPower(rp * robot.DRIVE_RATIO_FR);
                robot.motorFrontLeft.setPower(lp * robot.DRIVE_RATIO_FL);
                if (!robot.use_minibot) {
                    robot.motorBackLeft.setPower(lp * robot.DRIVE_RATIO_BL);
                    robot.motorBackRight.setPower(rp * robot.DRIVE_RATIO_BR);
                }
            } else {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                if (!robot.use_minibot) {
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
            robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB && strafeRight){
            robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB && !strafeRight){
            robot.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        if(!robot.fast_mode && robot.straight_mode) { // expect to go straight
            if (robot.use_imu) {
                double cur_heading = imu_heading();
                double heading_off_by = ((cur_heading - robot.target_heading) / 360);
                if(robot.use_swerve) {
                    if(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                        if(rp > 0 && lp > 0) { //When going forward
                            if(true) {
                                if (cur_heading - robot.target_heading > 0.7) {
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION - heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION - heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) {
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION + heading_off_by);
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION + heading_off_by);
                                }
                            }
                        }
                        else{ // When going backward
                            if(true) {
                                if (cur_heading - robot.target_heading > 0.7) { //Drifting to the left
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION + heading_off_by); //Turn Front servos to the right
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION + heading_off_by);
                                } else if (cur_heading - robot.target_heading < -0.7) { //Drifting to the right
                                    robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION - heading_off_by); //Turn Front servos to the left
                                    robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION - heading_off_by);
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
        if(robot.use_swerve) {
            if (robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT || robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                robot.motorBackLeft.setPower(0);
                robot.motorBackRight.setPower(0);

            } else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB && strafeRight) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(0);
                robot.motorBackLeft.setPower(0);
                robot.motorBackRight.setPower(-lp);
            }
            else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB && !strafeRight) {
                robot.motorFrontRight.setPower(0);
                robot.motorFrontLeft.setPower(-rp);
                robot.motorBackLeft.setPower(lp);
                robot.motorBackRight.setPower(0);
            }
            else if(robot.cur_mode == SwerveDriveHardware.CarMode.TURN) {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                robot.motorBackLeft.setPower(lp);
                robot.motorBackRight.setPower(rp);
            }
        }
        else{
            if (Math.abs(rp) > 0.3 && Math.abs(lp) > 0.3) {
                robot.motorFrontRight.setPower(rp * robot.DRIVE_RATIO_FR);
                robot.motorFrontLeft.setPower(lp * robot.DRIVE_RATIO_FL);
                if (!robot.use_minibot) {
                    robot.motorBackLeft.setPower(lp * robot.DRIVE_RATIO_BL);
                    robot.motorBackRight.setPower(rp * robot.DRIVE_RATIO_BR);
                }
            } else {
                robot.motorFrontRight.setPower(rp);
                robot.motorFrontLeft.setPower(lp);
                if (!robot.use_minibot) {
                    robot.motorBackLeft.setPower(lp);
                    robot.motorBackRight.setPower(rp);
                }
            }
        }
    }

    void stop_chassis() {
        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        if (!robot.use_minibot) {
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);
        }
    }

    void reset_chassis()  {
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (!robot.use_minibot) {
            robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        robot.leftCnt = 0;
        robot.rightCnt = 0;
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!robot.use_minibot) {
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    void stop_auto() {
        if (robot.use_color_sensor) {
            robot.colorSensor.enableLed(false);
            //robot.colorSensor.close();
            //robot.use_color_sensor = false;
        }
        if (robot.use_Vuforia) {
            robot.relicTrackables.deactivate();
            robot.use_Vuforia = false;
        }
        if (robot.use_camera) {
            robot.use_camera = false;
        }

    }

    void stop_tobot() {
        if (robot.use_swerve||robot.use_minibot)
            stop_chassis();
        if (robot.use_color_sensor) {
            robot.colorSensor.enableLed(false);
            robot.colorSensor.close();
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
        int curPosBackLeft = robot.motorBackLeft.getCurrentPosition();
        int targetPosBackRight;
        int curPosBackRight = robot.motorBackRight.getCurrentPosition();
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
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            while (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && (robot.runtime.seconds() < 7) && opModeIsActive()) {
                driveTTCoast(leftPower, rightPower);
            }

            if (rightTC2 > 0 || leftTC2 > 0) {
                curPosFrontLeft = robot.motorFrontLeft.getCurrentPosition();
                curPosFrontRight = robot.motorFrontRight.getCurrentPosition();

                targetPosFrontLeft = curPosFrontLeft + ((int) leftPowerSign * leftTC2);
                targetPosFrontRight = curPosFrontRight + ((int) rightPowerSign * rightTC2);

                robot.motorFrontLeft.setTargetPosition(targetPosFrontLeft);
                robot.motorFrontRight.setTargetPosition(targetPosFrontRight);
                driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                while (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && (robot.runtime.seconds() < 8) && opModeIsActive()) {
                    driveTTCoast(leftPowerSign * 0.2, rightPowerSign * 0.2);
                }
            }
        }
        else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB){
            if(strafeRight) {
                if (rightTC0 > 0 || leftTC0 > 0) {
                    targetPosFrontRight = curPosFrontRight + ((int) leftPowerSign * leftTC0);
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

                targetPosFrontRight = curPosFrontRight + ((int) leftPowerSign * leftTC1);
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

                    targetPosFrontRight = curPosFrontRight + ((int) leftPowerSign * leftTC2);
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
                    targetPosFrontLeft = curPosFrontLeft + ((int) -leftPowerSign * leftTC0);
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

                targetPosFrontLeft = curPosFrontLeft + ((int) -leftPowerSign * leftTC1);
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

                    targetPosFrontLeft = curPosFrontLeft + ((int) -leftPowerSign * leftTC2);
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
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop_chassis();
        robot.runtime.reset();
    }

    boolean has_left_drive_encoder_reached(double p_count) {
        DcMotor mt = robot.motorFrontLeft;
        if(robot.use_swerve){
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
        if(robot.use_swerve) {
            if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB) {
                robot.servoFrontLeft.setPosition(robot.SERVO_FL_STRAFE_POSITION);
                robot.servoFrontRight.setPosition(robot.SERVO_FR_STRAFE_POSITION);
                robot.servoBackLeft.setPosition(robot.SERVO_BL_STRAFE_POSITION);
                robot.servoBackRight.setPosition(robot.SERVO_BR_STRAFE_POSITION);
            }
            else{
                robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
                robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
                robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
                robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);
            }
        }
        if (!robot.fast_mode)
            sleep(135);
    }

    void StraightIn(double power, double in) throws InterruptedException {
        if (robot.use_imu) {
            robot.target_heading = imu_heading();
        }
        if (robot.use_encoder) {
            double numberR = in / robot.INCHES_PER_ROTATION;
            StraightR(-power, numberR);
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
            StraightR(-power, numberR);
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

        sleep(300);

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
            while ((current_pos >= robot.target_heading) && (robot.runtime.seconds() < 4.0)) {
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
        sleep(300);
        change_swerve_pos(robot.old_mode);
        if (!robot.fast_mode)
            sleep(135);
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

        sleep(300);

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
            while ((current_pos <= robot.target_heading) && (robot.runtime.seconds() < 5.0)) {
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
        sleep(300);
        change_swerve_pos(robot.old_mode);
        if (!robot.fast_mode)
            sleep(135);
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
        if (isBlueAlliance) {
            robot.blue = robot.colorSensor.blue();
            robot.red = robot.colorSensor.red();
        } else {
            robot.blue = robot.r_colorSensor.blue();
            robot.red = robot.r_colorSensor.red();
        }
        return (robot.blue - robot.red);
    }

    enum RangeSensor{
        BACK, LEFT
    }

    /**
     * Function to prevent range sensor from returning an error
     * If this returns 6666666, it failed to get the right range in under .1 seconds
     * @param direction
     * @return
     */
    double getRange(RangeSensor direction){
        ElapsedTime elapsedTime = new ElapsedTime();
        double distance = 6666666;
        if (!robot.use_range_sensor)
            return 0.0;
        if(direction == RangeSensor.BACK){
            while(distance > 365 && elapsedTime.seconds() < 0.1){
                distance = robot.rangeSensorBack.getDistance(DistanceUnit.CM);
            }
        }
        else if(direction == RangeSensor.LEFT){
            while(distance > 365 && elapsedTime.seconds() < 0.1){
                distance = robot.rangeSensorLeft.getDistance(DistanceUnit.CM);
            }
        }
        else {
            throw new IllegalArgumentException("Direction not specified!");
        }
        return distance;
    }

    /**
     * doPlatformMission is meant to execute everything that needs to be done on the starting platform, including
     * determining what the bonus column is for the cryptobox from the pictograph, determining the jewel colors and
     * knocking off the opposing alliance ball based on the colors from the color class.
     * @param isBlueAlliance
     * @throws InterruptedException
     */
    public void doPlatformMission(boolean isBlueAlliance) throws InterruptedException {

        //These constants are for setting a selected portion of the image from Camera
        //(Assuming portrait) Top left is (0,0), Top right (0,1), Bottom left is (1,0), Bottom right is (1,1)
        double IMAGE_WIDTH_CROP = 1;
        double IMAGE_HEIGHT_CROP = 1;
        double IMAGE_OFFSET_X = 0; // Cannot be 1, make sure take the respective crop into consideration
        double IMAGE_OFFSET_Y = 0; // Cannot be 1, make sure take the respective crop into consideration


        if (robot.targetColumn<0) {
            robot.targetColumn = get_cryptobox_column();
        }
        TeamColor rightJewelColorCamera = TeamColor.UNKNOWN;

        ElapsedTime runTime = new ElapsedTime();


        if(robot.use_camera) {
            Bitmap bitmap = null;
            robot.camera.activate();
            while (runTime.seconds() < 0.5) {
                bitmap = robot.camera.captureBitmap(IMAGE_OFFSET_X, IMAGE_OFFSET_Y, IMAGE_WIDTH_CROP, IMAGE_HEIGHT_CROP);
                if (bitmap != null) {
                    break;
                }
            }
            if (bitmap == null) {
                telemetry.addData("Warning!", robot.camera.getLastError()).setRetained(true);
                telemetry.update();

                //while (opModeIsActive());
            } else {
                TeamColor leftJewelColorCamera = determineJewelColor(bitmap);
                telemetry.addData("Left Jewel Color", leftJewelColorCamera);
                //Mirrors to allow checking of right jewel
                rightJewelColorCamera = leftJewelColorCamera.getOpposingColor();
                //Current mounting solution only allows camera to check the left jewel color
                robot.camera.stopCamera();
            }
        }


        if (robot.use_glyph_grabber) {
            glyph_grabber_close();
            sleep(100);
            glyph_slider_up_inches(.5, 4);
        }
        if (robot.allianceColor == TeamColor.BLUE) {
            arm_down();
        } else {
            r_arm_down();
        }
        sleep(1000);

        TeamColor rightJewelColorCS = checkBallColor(isBlueAlliance);

        // Comparing of sensor data
        boolean isRedBall = (rightJewelColorCS == TeamColor.RED);
        boolean isBlueBall = (rightJewelColorCS == TeamColor.BLUE);

        telemetry.addData("isBlueBall/isRedBall", "%s/%s", isBlueBall, isRedBall);
        telemetry.update();


        // Determines if right jewel is red
        int directionI = calcArmDirection(rightJewelColorCS, rightJewelColorCamera, isBlueAlliance);
        if (isBlueAlliance) {
            if (directionI == 1) { // Right jewel is our color
                arm_left();
            } else if (directionI == -1) { // Right jewel is their color
                arm_right();
            }
            // if directionI 0, then color is unknown
        } else {
            StraightCm(.1 * directionI, 5); // Drives forward if right jewel is red, backwards if blue
            sleep(100);
            StraightCm(-.1 * directionI, 5); // Drives forward if right jewel is blue, backwards if red
        }

        sleep(1000);
        if (isBlueAlliance) {
            arm_up();
        } else {
            r_arm_up();
        }
    }

    // Provided two colors guesses and team color, determines if the right ball is the same color as our team.
    // if sum = 1, then it is our color. if sum == 0, then it is unknown. if sum = -1, then it is their color.
    // Ex. sensor is red, camera is unknown, team is red. It will return 1.
    public int calcArmDirection(TeamColor sensor, TeamColor camera, boolean isBlueAlliance) {
        int sum = 0;
        if (sensor == TeamColor.RED || camera == TeamColor.RED) sum += 1;
        if (sensor == TeamColor.BLUE || camera == TeamColor.BLUE) sum -= 1;
        if (isBlueAlliance) sum *= -1;
        return sum;
    }

    public void deliverGlyph() throws InterruptedException{
        StraightIn(0.3, 7);

        glyph_grabber_open_and_push_auto();
        // glyph_grabber_half_close();
        StraightIn(-0.4, 7);
        rotate_refine(); // ensure grabber back straight
        glyph_slider_back_init();
        // glyph_grabber_close();
        // sleep(500);
        // 0.4 will break the arms
        StraightIn(0.2, 10);
        sleep(100);
        StraightIn(-0.4, 3);
        glyph_grabber_auto_open();
    }

    TeamColor checkBallColor(boolean isBlueAlliance) throws InterruptedException {
        boolean isBlueBall = false;
        boolean isRedBall = false;
        robot.runtime.reset();
        double d = calcDelta(isBlueAlliance);
        TeamColor result = TeamColor.UNKNOWN;

        while (!isBlueBall && !isRedBall && (robot.runtime.seconds()<1.0)) {
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

    void arm_up() {
        robot.sv_elbow.setPosition(robot.SV_ELBOW_UP);
        sleep(200);

        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_INIT);
}

    void arm_down() {
        robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
        robot.sv_shoulder.setPosition(robot.SV_SHOULDER_DOWN);
    }

    void r_arm_up() {
        robot.sv_right_arm.setPosition(robot.SV_RIGHT_ARM_UP);
        sleep(200);
    }

    void r_arm_down() {
        robot.sv_right_arm.setPosition(robot.SV_RIGHT_ARM_DOWN);
        sleep(200);
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
    void go_to_distance_from(double power, int targetColumn, boolean isBlue, boolean isSideBox, boolean use_encoder)
            throws InterruptedException {
        // Go until a certain distance from a target depending on the cryptobox and the column
        // use_encoder is true will use Motor encoder for the driving distance
        //                false will use range sensor for the driving distance
        // IMPORTANT NOTE: This program does not work on front red box without encoders.

        double driveDistance;

        if (targetColumn < 0) targetColumn = 1;
        if (isSideBox) {
            if(isBlue) { // Side Blue
                driveDistance = 7 + (18 * targetColumn); // 19cm between columns
            }
            else { // Side Red
                driveDistance = 7 + (18 * (2 - targetColumn)); // 19cm between columns
            }
            robot.runtime.reset(); // Rest of side
            if (use_encoder) {
                StraightCm(power, driveDistance);
            } else {
                double cur_dist = robot.rangeSensorBack.getDistance(DistanceUnit.CM);
                driveTT(-1 * power, -1 * power); // Drives to the right
                while ((cur_dist <= driveDistance - 7) && robot.runtime.seconds() < 4) { // Waits until it has reached distance
                    cur_dist = robot.rangeSensorBack.getDistance(DistanceUnit.CM);
                }
                if (cur_dist <= driveDistance && robot.runtime.seconds() < 4) {
                    driveTT(power / 2, power / 2); // Drives to the left, slower
                    while (cur_dist >= driveDistance && robot.runtime.seconds() < 4) { // Waits until it has reached distance
                        cur_dist = robot.rangeSensorBack.getDistance(DistanceUnit.CM);
                    }
                }
            }
        } else { // Front box
            if (isBlue) { // Front Blue
                if (use_encoder) {
                    driveDistance = 6 + (19 * targetColumn); // 19cm between columns
                } else {
                    driveDistance = 52 + (19 * targetColumn); // 19cm between columns
                }
                robot.runtime.reset();
                change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
                sleep(1000);
                if (use_encoder) {
                    if(isBlue){
                        StraightCm(power, driveDistance);
                    }
                    else{
                        StraightCm(-power, driveDistance);
                    }
                } else {
                    double cur_dist = robot.rangeSensorLeft.getDistance(DistanceUnit.CM);
                    driveTT(-1 * power, -1 * power); // Drives to the right
                    while ((cur_dist <= driveDistance - 20) && (robot.runtime.seconds() < 15)) { // Waits until within 10 cm
                        cur_dist = robot.rangeSensorLeft.getDistance(DistanceUnit.CM);
                    }
                    driveTT(0, 0);
                    driveTT(-1 * power / 2, -1 * power / 2); // Slows to half speed
                    while (cur_dist <= driveDistance - 5 && robot.runtime.seconds() < 15) { // Waits until within 4 cm
                        cur_dist = robot.rangeSensorLeft.getDistance(DistanceUnit.CM);
                    }
                    driveTT(0, 0);
                    cur_dist = robot.rangeSensorLeft.getDistance(DistanceUnit.CM);
                    if (cur_dist < driveDistance) {
                       driveTT(-1 * power / 3, -1 * power / 3); // Slows to third speed
                       while (cur_dist <= driveDistance && robot.runtime.seconds() < 20) { // Waits until it has reached distance
                           cur_dist = robot.rangeSensorLeft.getDistance(DistanceUnit.CM);
                       }
		            }
                }
            }
            else { // Front red box
                power *= -1; // Reverses power input, all other code is pretty much the same
                if (use_encoder) {
                    driveDistance = 25 + (19 * (2 - targetColumn)); // 19cm between columns
                } else {
                    driveDistance = 52 + (19 * (2 - targetColumn)); // 19cm between columns
                }
                robot.runtime.reset();
                change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
                sleep(500);
                if (use_encoder) {
                    StraightCm(power, driveDistance);
                } else {
                    // Do nothing, there is no range sensor for this side.
                }
            }
        }
        driveTT(0, 0); // Stops
        change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
        sleep(300);
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
        float stick_x = 0;
        if (left_t > 0.1)
            stick_x = -1 * (float)(left_t / 1.5);
        else stick_x = (float)(right_t / 1.5);
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

        robot.thetaOneCalc = (Math.atan((0.5 * robot.WIDTH_BETWEEN_WHEELS) / ((robot.r_Value) - (0.5 * robot.LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 1
        robot.thetaTwoCalc = (Math.atan((0.5 * robot.WIDTH_BETWEEN_WHEELS) / ((robot.r_Value) + (0.5 * robot.LENGTH_BETWEEN_WHEELS))) / (Math.PI)) + 0.5; //Theta 2
    }

    void alignUsingIMU() throws InterruptedException {
        double imu = imu_heading();
        if (imu < -0.5) {
            TurnLeftD(.1, Math.abs(imu));
        }
        else if (imu > 0.5) {
            TurnRightD(.1, Math.abs(imu));
        }
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
        else{
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
            robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);
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
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
            robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);
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
        else if(new_mode == SwerveDriveHardware.CarMode.STRAIGHT){
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
            robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);

            robot.servoPosFL = robot.SERVO_FL_FORWARD_POSITION;
            robot.servoPosFR = robot.SERVO_FR_FORWARD_POSITION;
            robot.servoPosBL = robot.SERVO_BL_FORWARD_POSITION;
            robot.servoPosBR = robot.SERVO_BR_FORWARD_POSITION;
        }
        else if(new_mode == SwerveDriveHardware.CarMode.CAR){
            robot.servoFrontLeft.setPosition(robot.SERVO_FL_FORWARD_POSITION);
            robot.servoFrontRight.setPosition(robot.SERVO_FR_FORWARD_POSITION);
            robot.servoBackLeft.setPosition(robot.SERVO_BL_FORWARD_POSITION);
            robot.servoBackRight.setPosition(robot.SERVO_BR_FORWARD_POSITION);

            robot.servoPosFL = robot.SERVO_FL_FORWARD_POSITION;
            robot.servoPosFR = robot.SERVO_FR_FORWARD_POSITION;
            robot.servoPosBL = robot.SERVO_BL_FORWARD_POSITION;
            robot.servoPosBR = robot.SERVO_BR_FORWARD_POSITION;
        }
        robot.old_mode = robot.cur_mode;
        robot.cur_mode = new_mode;

        robot.isTestingFL = false;
        robot.isTestingFR = false;
        robot.isTestingBL = false;
        robot.isTestingBR = false;
    }

    // void set_swerve_power(float right_stick, float left_stick, float x_stick){
    void set_swerve_power(float right_stick, float left_stick, float left_t, float right_t){
        float x_stick = 0;
        if (left_t > 0.1)
            x_stick = -1 * left_t;
        else x_stick = right_t;

        if(robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
            robot.insideWheelsMod = left_stick * ((Math.pow((Math.pow(0.5 * robot.LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) - robot.WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                    (robot.r_Value));
            robot.outsideWheelsMod = left_stick * ((Math.pow((Math.pow(0.5 * robot.LENGTH_BETWEEN_WHEELS, 2) + Math.pow((robot.r_Value) + robot.WIDTH_BETWEEN_WHEELS, 2)), 0.5)) /
                    (robot.r_Value));
            if (robot.enoughToSnake) {
                robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                if (robot.isSnakingLeft) {
                    robot.motorPowerLeft = robot.insideWheelsMod;
                    robot.motorPowerRight = robot.outsideWheelsMod;
                } else {
                    robot.motorPowerLeft = robot.outsideWheelsMod;
                    robot.motorPowerRight = robot.insideWheelsMod;
                }
            } else {
                robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                robot.motorPowerLeft = left_stick;
                robot.motorPowerRight = left_stick;
            }


            robot.motorFrontLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
            robot.motorFrontRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
            if(robot.enoughToSnake){
                robot.motorBackLeft.setPower(0);
                robot.motorBackRight.setPower(0);
            }
            else {
                robot.motorBackLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                robot.motorBackRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
            }

        } else if (robot.cur_mode == SwerveDriveHardware.CarMode.TURN) {
            robot.motorPowerTurn = x_stick;
            robot.motorFrontLeft.setPower(-robot.motorPowerTurn * robot.drivePowerRatio);
            robot.motorFrontRight.setPower(robot.motorPowerTurn * robot.drivePowerRatio);
            robot.motorBackLeft.setPower(-robot.motorPowerTurn * robot.drivePowerRatio);
            robot.motorBackRight.setPower(robot.motorPowerTurn * robot.drivePowerRatio);
        } else {
            robot.motorPowerLeft = left_stick;
            robot.motorPowerRight = right_stick;
            if (robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT) {
                robot.motorFrontLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                robot.motorFrontRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
                robot.motorBackLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                robot.motorBackRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
            } else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB) {
                robot.motorFrontLeft.setPower(-robot.motorPowerRight * robot.drivePowerRatio);
                robot.motorFrontRight.setPower(robot.motorPowerRight * robot.drivePowerRatio);
                robot.motorBackLeft.setPower(robot.motorPowerLeft * robot.drivePowerRatio);
                robot.motorBackRight.setPower(-robot.motorPowerLeft * robot.drivePowerRatio);
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
            lastError = "";
            int capacity = vuforia.getFrameQueueCapacity();
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().poll();

            if (frame == null) {
                lastError = "ERROR! Failed to retrieve frame!";
                return null;
            }

            Bitmap bitmapTemp = convertFrameToBitmap(frame);

            frame.close();
            if (bitmapTemp == null) {
                lastError = "ERROR! Failed to retrieve bitmap";
                return null;

            }
            //White Balance applied here
            //int whitestPixel = getWhitestPixel(bitmapTemp);
            //applyWhiteBalance(bitmapTemp, whitestPixel);

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
            bitmap.getPixels(pixels, 0, bitmap.getWidth(), 0, bitmap.getHeight() * 5 / 6, bitmap.getWidth(), 1);
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

            if(pixelRed > pixelGreen * 1.4 && pixelRed > pixelBlue * 1.4 && greenToRedRatio < .3){
                redCount++;
                if (redStart == -1){
                    redStart = pixelI;
                }
            }
            else if (pixelBlue > pixelRed * 1.4 && greenToBlueRatio > .65 && greenToBlueRatio < 1){
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


    void show_telemetry() throws InterruptedException {
        telemetry.addData("1. Team", " %s sw/IMU/Vu/GG = %s/%s/%s/%s",
                robot.allianceColor, (robot.use_swerve ?"Y":"N"), (robot.use_imu?"Y":"N"),
                (robot.use_Vuforia ?"Y":"N"), (robot.use_glyph_grabber ?"Y":"N"));
        telemetry.addData("2. PW-r/L/R/Rot-En/Sl-En =", "%.2f,%.2f/%.2f/%s/%s",
                robot.drivePowerRatio, robot.motorPowerLeft,robot.motorPowerRight,
                (robot.gg_rotator_encoder_ok ?"Y":"N"),(robot.gg_slider_encoder_ok ?"Y":"N"));
        telemetry.addData("3. W-sv angle FL/FR/BL/BR =", "%.3f/%.3f/%.3f/%.3f",
                robot.servoPosFL, robot.servoPosFR, robot.servoPosBL, robot.servoPosBR);
        double range_back = getRange(RangeSensor.BACK);
        double range_left = getRange(RangeSensor.LEFT);
        if (robot.use_imu) {
            telemetry.addData("4.1 IMU/r-Bk/r-Lf = ", "%.2f/%.2f/%.2f (cm)",
                    imu_heading(),range_back,range_left);
        }

        if (robot.use_Vuforia) {
            telemetry.addData("5. Vuforia Column = ", "%d", get_cryptobox_column());
        }
        if (robot.use_swerve && !robot.servo_tune_up) {
            telemetry.addData("6. Drive Mode = ", "%s", robot.cur_mode);
            telemetry.addData("6.1 SW-Enc FL/FR/BL/BR = ", "%d/%d/%d/%d",
                    robot.motorFrontLeft.getCurrentPosition(),
                    robot.motorFrontRight.getCurrentPosition(),
                    robot.motorBackLeft.getCurrentPosition(),
                    robot.motorBackRight.getCurrentPosition());
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
        if (robot.use_relic_grabber) {
            telemetry.addData("9.1 relic gr/arm = ", "%4.4f/%4.3f",
                    robot.sv_relic_grabber.getPosition(), robot.sv_relic_arm.getPosition());
        }
        if (robot.use_relic_slider) {
            telemetry.addData("9.2 r-slider pwr/enc = ","%3.2f/%d",
                    robot.mt_relic_slider.getPower(),robot.mt_relic_slider.getCurrentPosition());
        }
        telemetry.update();
    }
}
