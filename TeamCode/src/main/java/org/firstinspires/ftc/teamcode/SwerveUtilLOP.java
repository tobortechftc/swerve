package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.hardware.SwerveSystem;

import static org.firstinspires.ftc.teamcode.hardware.SwerveSystem.CarMode.CAR;
import static org.firstinspires.ftc.teamcode.hardware.SwerveSystem.CarMode.CRAB;


public abstract class SwerveUtilLOP extends LinearOpMode {

    /* Declare OpMode members. */
      public SwerveDriveHardware robot           = new SwerveDriveHardware();

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

    /*
     **** [ Initialization code (called just once at start) ] ****
     */

    public void init_and_test() throws InterruptedException {

        // Create a Supplier object that will invoke 'opModeIsActive' calls...
        Supplier<Boolean> supplierForOpModeIsActive = new Supplier<Boolean>() {
            @Override
            public Boolean get() {
                // Indicate if this OpMode is active.
                return opModeIsActive();
            }
        };
        // ... and pass it along to the robot, as its "shall I continue operating" master control.

        robot.init(hardwareMap, telemetry, supplierForOpModeIsActive, this);

//        if (robot.use_glyph_grabber) {
//            // test_glyph_rotator_encoder();
//            robot.gg_rotator_encoder_ok = true;
//            // test_glyph_slider_encoder();
//            robot.gg_slider_encoder_ok = true;
//        }
        if (robot.use_dumper) {
            robot.gg_slider_encoder_ok = true;
        }
    }

    public void start_init() throws InterruptedException {
        robot.runtimeAuto.reset();
//        if (robot.use_glyph_grabber) {
//            glyph_grabber_auto_open();
//            glyph_slider_init();
//        }
        //if (robot.use_dumper) {
        //    lift_to_target(robot.LIFT_INIT_COUNT);
        //}
        if (robot.swerve.use_newbot_v2 && robot.use_arm && (robot.sv_jkicker!=null)) {
            robot.sv_jkicker.setPosition(robot.SV_JKICKER_UP);
        }
        if (robot.relicReachSystem.use_relic_grabber) {
            if (robot.swerve.use_newbot) {
                robot.relicReachSystem.relic_grabber_open(false);
            } else {
                robot.relicReachSystem.relic_grabber_open(false);
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
        if (robot.swerve.use_imu) {
            if (robot.swerve.imu.getSystemStatus()== BNO055IMU.SystemStatus.SYSTEM_ERROR && robot.swerve.imu2!=null) {
                robot.swerve.use_imu2 = true;
            }
        }
        if (robot.use_verbose)
            telemetry.addData("0: End start_init CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
    }

    /*
     *    [ 0. Shared System ] ****
     */

    /*
     **** [ 2. Glyph Intake System ] ****
     */

    // [This method is invoked within TaintedAccess]
    public void intakeGateInit() {
        if (!robot.use_intake || !robot.swerve.use_newbot_v2)
            return;
        robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_INIT);
    }

    void intakeGateUp() {
        if (!robot.use_intake || !robot.swerve.use_newbot_v2)
            return;
        double pos = robot.sv_intake_gate.getPosition();
        if (Math.abs(pos-robot.SV_INTAKE_GATE_DOWN)<0.1)
            robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_MID);
        else if (Math.abs(pos-robot.SV_INTAKE_GATE_MID)<0.1)
            robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_UP);
        else
            robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_INIT);
    }

    void intakeGateDown() {
        if (!robot.use_intake || !robot.swerve.use_newbot_v2)
            return;
        if (robot.swerve.use_newbot_v2 && robot.relicReachSystem.use_relic_grabber)
            robot.relicReachSystem.relic_arm_up();
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


    /*
     **** [ 3. Glyph Dump System ] ****
     */
    public void dumper_vertical() {
        if (!robot.use_dumper)
            return;
        robot.sv_dumper.setPosition(robot.SV_DUMPER_UP);
        if (robot.use_dumper_gate) {
            dumperGateDown();
        }
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

    void lift_back_init() { // back to initial position
        robot.target_gg_slider_pos = robot.init_gg_slider_pos;
        slide_to_target(robot.GG_SLIDE_DOWN_POWER);
        dumper_down(true);
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

    public void slide_to_target(double power) {
        if (!robot.gg_slider_encoder_ok)
            return;

        robot.swerve.stop_chassis(); // ensure chassis stops

        if (power<0) power=-1.0*power;
        DcMotor mt = (robot.swerve.use_newbot? robot.mt_lift:robot.mt_glyph_slider);

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

    void dumperGateUp() {
        if (!robot.use_dumper || !robot.swerve.use_newbot_v2)
            return;
        robot.sv_dumper_gate.setPosition(robot.SV_DUMPER_GATE_UP);
    }

    void dumperGateDown() {
        if (!robot.use_dumper || !robot.swerve.use_newbot_v2)
            return;
        robot.sv_dumper_gate.setPosition(robot.SV_DUMPER_GATE_DOWN);
    }


    /*
     **** [ 4. Jewel Arm System ] ****
     */

    void l_arm_up() {
        if (robot.swerve.use_newbot) {
            robot.sv_left_arm.setPosition(robot.SV_LEFT_ARM_UP_NB);
        } else {
            robot.sv_elbow.setPosition(robot.SV_ELBOW_UP);
            sleep(200);
            robot.sv_shoulder.setPosition(robot.SV_SHOULDER_INIT);
        }
    }

    void l_arm_down() {
        if (robot.swerve.use_newbot) {
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
        if (robot.swerve.use_newbot) {
            robot.sv_right_arm.setPosition(robot.SV_RIGHT_ARM_UP_NB);
        } else {
            robot.sv_right_arm.setPosition(robot.SV_RIGHT_ARM_UP);
        }
        sleep(200);

    }

    void r_arm_down() {
        if (robot.swerve.use_newbot) {
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
        sleep(250);
    }

    void jkick_left() {
        if (robot.sv_jkicker==null) return;
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_LEFT1);
        sleep(100);
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_LEFT2);
        sleep(150);
        robot.sv_jkicker.setPosition(robot.SV_JKICKER_LEFT);
        sleep(250);
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

    /*
     **** [ 5. Camera System ] ****
     */

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

    /*
     **** [ 7. Autonomous or TeleOp Mission  ]
     */

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
        if (robot.swerve.use_newbot) {
            if (isBlueAlliance || robot.swerve.use_newbot_v2) {
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

        if (robot.swerve.use_newbot) {
            if (robot.allianceColor == TeamColor.BLUE || robot.swerve.use_newbot_v2) {
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

//        if (robot.use_glyph_grabber) {
//            glyph_grabber_close();
//            sleep(100);
//            glyph_slider_up_inches(.5, 3);
//        }

        if (robot.swerve.use_newbot_v2)
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
        if (robot.swerve.use_newbot_v2 && !isBlueAlliance) {
            directionI *= -1;
        }
        if (!opModeIsActive()) return 0;
        if (robot.sv_jkicker!=null) {
            if (directionI<0)
                jkick_left();
            else if (directionI>0)
                jkick_right();
            r_arm_up();
        } else if (robot.swerve.use_newbot) {
            int dist = (directionI > 0 ? 7 : 6);
            robot.swerve.StraightCm(-.2 * directionI, dist); // Drives forward if right jewel is red, backwards if blue
            sleep(100);
            if (isBlueAlliance || robot.swerve.use_newbot_v2)
                r_arm_up(); // arm up to ensure the jewel is knocked out
            else
                l_arm_up(); // arm up to ensure the jewel is knocked out
            // next_dist = -1.0*dist*directionI;
            robot.swerve.StraightCm(.4 * directionI, dist); // Drives forward if right jewel is blue, backwards if red
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
                robot.swerve.StraightCm(.1 * directionI, dist); // Drives forward if right jewel is red, backwards if blue
                sleep(100);
                r_arm_up(); // arm up to ensure the jewel is knocked out
                robot.swerve.StraightCm(-.15 * directionI, dist); // Drives forward if right jewel is blue, backwards if red
            }
        }
        if (!opModeIsActive()) return next_dist;

        if (isBlueAlliance && !robot.swerve.use_newbot) {
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
        robot.stop_on_bump = true;
    }

    void disable_bump_detection() {
        robot.bump_detected = false;
        robot.stop_on_bump = false;
    }

    public void deliverGlyph() throws InterruptedException{
        if (!opModeIsActive()) return;
        if (robot.swerve.use_newbot) {
            robot.swerve.StraightIn(0.6, 2);
            if (!opModeIsActive()) return;
            dumper_vertical();
            // dumper_up();
            if (!opModeIsActive()) return;
            sleep(500);
            if (!opModeIsActive()) return;
            robot.swerve.driveTT(0.4, 0.4); // drive backward for .3 sec
            sleep(250);
            robot.swerve.driveTT(0,0);
            if (!opModeIsActive()) return;
            robot.swerve.StraightIn(0.8, 6);
            dumper_down(true);
        }
        if (robot.use_verbose)
            telemetry.addData("0: End deliveryGlyph() CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
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

    void go_to_crypto(double next_dist, double power, int targetColumn, boolean isBlue, boolean isSideBox)throws InterruptedException {
        if (targetColumn < 0) targetColumn = 1;
        boolean nearest_col = false;
        if ((targetColumn==0 && isBlue==true) || (targetColumn==2 && isBlue==false))
            nearest_col = true;

        int direction = (robot.swerve.use_newbot_v2&&!isBlue?1:-1);
        if (robot.swerve.use_newbot) {
            double dist = next_dist + (isSideBox ? 20 : 24);
            if (nearest_col)
                dist += 3;
            robot.swerve.StraightIn(direction*.3, dist); // Drive off the balance stone
        } else {
            robot.swerve.StraightIn(-.25, (isSideBox ? 22 : 24)); // Drive off the balance stone
        }
        if (direction==1 && !isSideBox) { // red front need to turn 180 degree
            robot.swerve.TurnLeftD(0.4, 180);
            robot.swerve.alignUsingIMU(0.18, 178.0);
        } else {
            robot.swerve.alignUsingIMU(0.18, 0);
        }

        double driveDistance;
        double dist;

        if (isSideBox) {
            if (isBlue) driveDistance = 6 + (18.5 * targetColumn); // 18cm between columns
            else driveDistance = 6 + (18.5 * (2 - targetColumn));
            if (driveDistance<7) driveDistance=7; // ensure turn not hitting balance stone
            robot.swerve.StraightCm(direction*power, driveDistance); // drive to cryptobox

            if (isBlue || robot.swerve.use_newbot_v2) {
                robot.swerve.TurnLeftD(0.35, 90);
                robot.swerve.alignUsingIMU(0.18, 90.0);
            } else {
                robot.swerve.TurnRightD(0.35, 90);
                robot.swerve.alignUsingIMU(0.18, -90.0);
            }
            if (!opModeIsActive()) return;
            enable_bump_detection();
            for(int i = 0 ; i < 3 ; i++) {
                dist = Math.max(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 15;
                if (dist > 30) dist = 7;
                if (dist<1) break;
                robot.swerve.StraightCm(-.3, dist); // drive close to cryptobox
            }
            if (robot.bump_detected) {
                // hit divider, forward 2 cm
                robot.bump_detected=false;
                robot.swerve.StraightCm(.3, 2);
            }
            disable_bump_detection();
            if (isBlue || robot.swerve.use_newbot_v2) {
                robot.swerve.alignUsingIMU(0.18, 90.0);
            } else {
                robot.swerve.alignUsingIMU(0.18, -90.0);
            }
        } else { // Front box
            if (isBlue) { // Front Blue
                driveDistance = 0 + (18 * targetColumn); // 18cm between columns
            } else { // Front Red
                driveDistance = 7 + (18.5 * (2 - targetColumn)); // 18.5 cm between columns
            }
            if (!opModeIsActive()) return;

            robot.swerve.change_swerve_pos(CRAB);
            sleep(200);
            if (isBlue)
                robot.swerve.StraightCm(-power, driveDistance);
            else
                robot.swerve.StraightCm(power, driveDistance);
            robot.swerve.change_swerve_pos(CAR);
            if(isBlue){
                robot.swerve.StraightCm(-0.2, 10);
            }
            else {
                robot.swerve.StraightCm(-.2,5);
            }

            enable_bump_detection();
            for (int i=1; i<3; i++) {
                dist = Math.max(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 15;
                if (dist<2) break;
                if (dist > 50 || (dist <= 5 && i==0)) {
                    robot.swerve.StraightCm(-.25, 16);
                } else if (dist > 1) {
                    robot.swerve.StraightCm(-.25, dist); // forward using front range sensor, so it is close to cryptobox
                }
            }
            if (robot.bump_detected) {
                // hit divider, forward 2 cm
                robot.bump_detected=false;
                robot.swerve.StraightCm(.3, 2);
            }
            disable_bump_detection();
            robot.swerve.alignUsingIMU(0.18, (isBlue?0:178));
        }

        robot.swerve.change_swerve_pos(CRAB);
        // sleep(200);
        if (!opModeIsActive()) return;
        reset_prox();
        //if (nearest_col) {
        //    // allow more time for proximity sensor to reset
        //    sleep(400);
        //}
        if (isBlue) {
            robot.swerve.driveTT(0.15, 0.15); // Crabs to the left
        } else { // Red
            robot.swerve.driveTT(-0.15, -0.15); // Crabs to the right
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

        robot.swerve.driveTT(0, 0); // Stops

        if (!isBlue) { // crab back 3cm to correct proximity over shoot
            robot.swerve.StraightCm(-.4, 2);
        } else {
            robot.swerve.StraightCm(.4, 2);
        }

        if (!opModeIsActive()) return;

        robot.swerve.change_swerve_pos(CAR);
        sleep(300);
        robot.swerve.StraightCm(.4, 3); // goes back so that delivery has enough space
        if (robot.use_verbose)
            telemetry.addData("0: End go_to_crypto() CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
    }

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
                if (redStart == -1 && pixelI > 250){
                    redStart = pixelI;
                }
            }
            else if ((pixelBlue > (pixelRed * 1.4)) && (greenToBlueRatio > .65) && (greenToBlueRatio < .9)){
                blueCount++;
                if (blueStart == -1 && pixelI > 250){
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
    }

    void stop_tobot() {
        if (robot.swerve.use_swerve||robot.swerve.use_newbot)
            robot.swerve.stop_chassis();
        if (robot.use_color_sensor) {
            if (!robot.swerve.use_newbot_v2) {
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

    void reset_prox(){
        robot.proxL.setMode(DigitalChannel.Mode.OUTPUT);
        robot.proxR.setMode(DigitalChannel.Mode.OUTPUT);
        robot.proxL.setState(true);
        robot.proxR.setState(true);
        robot.proxL.setMode(DigitalChannel.Mode.INPUT);
        robot.proxR.setMode(DigitalChannel.Mode.INPUT);

        robot.proxFL.setMode(DigitalChannel.Mode.OUTPUT);
        robot.proxML.setMode(DigitalChannel.Mode.OUTPUT);
        robot.proxFL.setState(true);
        robot.proxML.setState(true);
        robot.proxFL.setMode(DigitalChannel.Mode.INPUT);
        robot.proxML.setMode(DigitalChannel.Mode.INPUT);
        sleep(200);
    }

    void show_telemetry() throws InterruptedException {
        telemetry.addData("1. Team", " %s sw/IMU/Vu/GG = %s%s/%s/%s/%s",
                robot.allianceColor, (robot.swerve.use_swerve||robot.swerve.use_newbot ?"Y":"N"),
                (robot.isTesting?"-T":""), (robot.swerve.use_imu?"Y":"N"),
                (robot.use_Vuforia ?"Y":"N"), (robot.use_glyph_grabber ?"Y":"N"));
        telemetry.addData("2. PW-r/L/R/Rot-En/Sl-En =", "%.2f,%.2f/%.2f/%s/%s",
                robot.swerve.drivePowerRatio, robot.swerve.motorPowerLeft,robot.swerve.motorPowerRight,
                (robot.gg_rotator_encoder_ok ?"Y":"N"),(robot.gg_slider_encoder_ok ?"Y":"N"));
        telemetry.addData("3. W-sv angle FL/FR/BL/BR =", "%.3f/%.3f/%.3f/%.3f",
                robot.swerve.servoPosFL, robot.swerve.servoPosFR, robot.swerve.servoPosBL, robot.swerve.servoPosBR);
        double range_front_left = getRange(RangeSensor.FRONT_LEFT);
        double range_front_right = getRange(RangeSensor.FRONT_RIGHT);
        double range_back = getRange(RangeSensor.BACK);
        boolean glyph_stuck = GlyphStuck();
        if (robot.swerve.use_imu||robot.use_range_sensor) {
            telemetry.addData("4.1 IMU/Range", "%s=%.2f(i2=%.0f)/L=%.0f/R=%.0f/B%s=%.0fcm",
                    (robot.swerve.use_imu2?"i2":"i1"),robot.swerve.imu_heading(),robot.swerve.imu_heading(), range_front_left,range_front_right,
                    (glyph_stuck?"(S)":""), range_back);
        }
        if (robot.use_proximity_sensor) {
            if (robot.swerve.use_newbot) {
                telemetry.addData("4.2.1 ProxSensorLeft =", robot.proxL.getState());
                telemetry.addData("4.2.2 ProxSensorRight =", robot.proxR.getState());
            }
            else {
                telemetry.addData("4.2 ProxSensor =", robot.proxL.getState());
            }
        }
        if (robot.use_color_sensor) {
            if (robot.swerve.use_newbot_v2) {
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
        if ((robot.swerve.use_swerve || robot.swerve.use_newbot)&& !robot.servo_tune_up) {
            telemetry.addData("6. Drive Mode = ", "%s", robot.swerve.cur_mode);
            telemetry.addData("6.1 SW-Enc FL/FR/BL/BR = ", "%d/%d/%d/%d",
                    robot.swerve.motorFrontLeft.getCurrentPosition(),
                    robot.swerve.motorFrontRight.getCurrentPosition(),
                    (robot.swerve.motorBackLeft!=null?robot.swerve.motorBackLeft.getCurrentPosition():0),
                    (robot.swerve.motorBackRight!=null?robot.swerve.motorBackRight.getCurrentPosition():0));
        }
        if (robot.use_intake) {
            telemetry.addData("7. Intake r = ", "%2.2f", robot.intakeRatio);
        }

        if (robot.use_dumper)  {
            telemetry.addData("8.1 dumper slide / pos = ","%d (pw=%.2f)/%3.3f",
                    robot.mt_lift.getCurrentPosition(), robot.mt_lift.getPower(),
                    robot.sv_dumper.getPosition());
        }

        if (robot.relicReachSystem.use_relic_grabber) {
            telemetry.addData("9.1 relic gr/wrist/elbow = ", "%4.4f/%4.3f/%4.3f",
                    robot.relicReachSystem.sv_relic_grabber.getPosition(), robot.relicReachSystem.sv_relic_wrist.getPosition(),
                    (robot.relicReachSystem.sv_relic_elbow!=null?robot.relicReachSystem.sv_relic_elbow.getPosition():0));
        }
        if (robot.relicReachSystem.use_relic_slider) {
            telemetry.addData("9.2 r-slider pwr/enc/tar = ","%3.2f/%d/%d",
                    robot.relicReachSystem.mt_relic_slider.getPower(),robot.relicReachSystem.mt_relic_slider.getCurrentPosition(),robot.target_relic_slider_pos);
        }
        if (robot.isTesting){
            if(robot.swerve.use_newbot){
                telemetry.addData("11. CRAB_LEFT_DIFF/CRAB_RIGHT_DIFF = ", "%.4f/%.4f", robot.swerve.NB_LEFT_SV_DIFF, robot.swerve.NB_RIGHT_SV_DIFF);
                telemetry.addData("12. CRAB_90_DIFF_DEC_FR/BR = ", "%.4f/%.4f",robot.swerve.NB_CRAB_DIFF_DEC_FR, robot.swerve.NB_CRAB_DIFF_DEC_BR);
                telemetry.addData("13. CRAB_90_DIFF_INC_FL/BL = ", "%.4f/%.4f", robot.swerve.NB_CRAB_DIFF_INC_FL, robot.swerve.NB_CRAB_DIFF_INC_BL);
            }
            else {
                telemetry.addData("11. CRAB_LEFT_DIFF/CRAB_RIGHT_DIFF = ", "%.4f/%.4f", robot.swerve.LEFT_SV_DIFF, robot.swerve.RIGHT_SV_DIFF);
            }
        }
        telemetry.update();
    }

    public void auto_relic_release() {
        robot.swerve.stop_chassis();
        double cur_pos = robot.relicReachSystem.sv_relic_wrist.getPosition();
        if (Math.abs(cur_pos - robot.relicReachSystem.SV_RELIC_WRIST_DOWN) > 0.1) {
            robot.relicReachSystem.relic_arm_down();
            sleep(500);
        }
        if (Math.abs(cur_pos - robot.relicReachSystem.SV_RELIC_WRIST_DOWN) > 0.02) {
            robot.relicReachSystem.relic_arm_down();
            sleep(500);
        }
        robot.relicReachSystem.relic_grabber_release();
        sleep(250);
        if (robot.swerve.use_newbot) {
            robot.relicReachSystem.relic_slider_in(0.5, true);
            sleep(400);
        } else {
            robot.relicReachSystem.relic_slider_in(1.0, true);
            sleep(1000);
        }
        robot.relicReachSystem.relic_slider_stop();
        robot.relicReachSystem.relic_arm_up();

    }

    public void relic_arm_auto() {
        robot.swerve.stop_chassis();
        if (robot.swerve.use_newbot) {
            // robot.relicReachSystem.sv_relic_wrist.setPosition(robot.SV_RELIC_WRIST_DOWN_AUTO);
            robot.relicReachSystem.relic_arm_down();
        } else {
            robot.relicReachSystem.relic_arm_down();
        }
    }

    /*
     **** [ UNDECIDED :( ] ****
     */

//    @Override
//    public void runOpMode() throws InterruptedException{
//        /* Initialize the hardware variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap, telemetry);
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Say", "Hello! Driver");    //
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//        }
//    }

    // returns true if it bumped the wall (noticeable negative acceleration), false if it went two seconds without hitting the wall
//    public boolean didBump() {
//        robot.accel = robot.imu.getAcceleration();
//        if (Math.abs(robot.accel.xAccel)+Math.abs(robot.accel.zAccel) >= 1.8) {
//            robot.bump_detected = true;
//            return true;
//        }
//        return false;
//    }

//    void front_arm_in() {
//        if (!robot.use_front_arm)
//            return;
//        robot.sv_front_arm.setPosition(robot.SV_FRONT_ARM_IN);
//    }
//
//    void front_arm_out() {
//        if (!robot.use_front_arm)
//            return;
//        robot.sv_front_arm.setPosition(robot.SV_FRONT_ARM_OUT);
//    }
//
//    void front_arm_sweep() {
//        stop_chassis();
//        front_arm_out();
//        sleep(300);
//        front_arm_in();
//    }

//    boolean has_left_drive_encoder_reached(double p_count) {
//        DcMotor mt = robot.motorFrontLeft;
//        if(robot.use_swerve||robot.use_newbot){
//            if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB){
//                if (-robot.leftPower < 0) {
//                    //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
//                    return (mt.getCurrentPosition() <= p_count);
//                } else {
//                    //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
//                    return (mt.getCurrentPosition() >= p_count);
//                }
//            }
//            else{
//                if (robot.leftPower < 0) {
//                    //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
//                    return (mt.getCurrentPosition() <= p_count);
//                } else {
//                    //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
//                    return (mt.getCurrentPosition() >= p_count);
//                }
//            }
//        }
//        else {
//            if (robot.leftPower < 0) {
//                //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
//                return (mt.getCurrentPosition() <= p_count);
//            } else {
//                //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
//                return (mt.getCurrentPosition() >= p_count);
//            }
//        }
//    } // has_left_drive_encoder_reached

//    boolean has_right_drive_encoder_reached(double p_count) {
//        DcMotor mt = robot.motorFrontRight;
//        if (robot.rightPower < 0) {
//            return (mt.getCurrentPosition() <= p_count);
//        } else {
//            return (mt.getCurrentPosition() >= p_count);
//        }
//
//    } // has_right_drive_encoder_reached

//    public void grabAndDump(boolean isSide, boolean isBlue) throws InterruptedException {
//        intakeGateMid();
//        double orig_imu = imu_heading();
//
//        if (opModeIsActive()==false ||
//                (isSide==true && robot.runtimeAuto.seconds() > 24) ||
//                (isSide==false && robot.runtimeAuto.seconds() > 22)) {
//            return;
//        }
//        if (opModeIsActive()) {
//            robot.fast_mode = true;
////            for(int i=0; i<2; i++) {
////                double distance = getRange(RangeSensor.BACK);
////                if (distance>20) {
////                    distance += 20;
////                    StraightCm((i==0?0.95:0.3), distance);
////                }
////            }
//            if (isSide)
//                StraightCm(.95, 75);
//            else
//                StraightCm(.95,115);
//            robot.fast_mode = false;
//        }
//        boolean got_one = autoIntakeGlyphs(isSide, isBlue);
//
//        if(isSide) alignUsingIMU(0.3, orig_imu);
//        else alignUsingIMU(0.3, orig_imu + (isBlue?14:-14));
//
//        if (opModeIsActive()) {
//            enable_bump_detection();
//            for (int i=0; i<2 && robot.bump_detected==false; i++) {
//                double dist = (isSide? Math.max(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 20:
//                        Math.min(getRange(RangeSensor.FRONT_LEFT), getRange(RangeSensor.FRONT_RIGHT)) - 20);
//                if (i==0) {
//                    if (isSide) {
//                        if (dist < 53) dist = 53;
//                        else if (dist > 80)
//                            dist = 80;
//                    } else { // front
//                        if (dist < 93) dist = 93;
//                        else if (dist > 120)
//                            dist = 120;
//                    }
//                    if (robot.runtimeAuto.seconds() > 27.5 || !got_one) {
//                        dist -= 15;
//                    }
//                } else if (dist<1) break;
//                if (i==0)robot.fast_mode = true;
//                StraightCm((i==0?-0.9:-0.5), dist);
//                robot.fast_mode =false;
//                if (robot.runtimeAuto.seconds() > 29) return;
//            }
//            if (robot.bump_detected) {
//                StraightCm(0.6,5);
//            }
//            disable_bump_detection();
//            StraightCm(0.6, 4);
//        }
//        if((robot.runtimeAuto.seconds() > 29 || !got_one) && !robot.servo_tune_up){
//            return;
//        }
//        else if (got_one && opModeIsActive()) {
//            //lift_up_level_half();
//            quickDump(isSide);
//            //lift_back_init();
////            if (alignBoxEdge() && opModeIsActive()) {
////                deliverGlyph();
////            }
//        }
//        if (robot.shxx.use_verbose)
//            telemetry.addData("0: End GrabAndDump() CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
//    }

//    public void quickDump(boolean isSide) throws InterruptedException {
//        double turn_left_angles = -5;
//        if (!opModeIsActive()) return;
//        dumper_vertical();
//        if (isSide) {
//            if (robot.targetColumn == 0) {
//                TurnRightD(0.6, 5);
//            } else {
//                TurnLeftD(0.6, 5);
//            }
//            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
//        }
//        else {
//            if (robot.allianceColor == TeamColor.RED) {
//                TurnLeftD(0.6, 3);
//            } else {
//                TurnRightD(0.6, 3);
//            }
//            change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
//        }
//        // dumper_up();
//        if (!opModeIsActive()) return;
//        sleep(300);
//        if (!opModeIsActive()) return;
//        if (robot.runtimeAuto.seconds() < 29 || robot.servo_tune_up==true) {
//            // sleep(100);
//            driveTT(0.6, 0.6); // drive backward for .2 sec
//            sleep(300);
//            driveTT(0, 0);
//            if (!opModeIsActive()) return;
//            StraightIn(0.9,3); // out 5.5 in
//            if (!opModeIsActive()) return;
//            if (robot.runtimeAuto.seconds() < 29.5)
//                sleep(100);
//            StraightIn(0.9,4); // out 5.5 in
//            if (!opModeIsActive()) return;
//            dumper_down(false);
//        } else {
//            StraightIn(0.9,5.5);
//        }
//        if (!opModeIsActive()) return;
//    }

//    public boolean autoIntakeGlyphs(boolean isSide, boolean isBlue) throws InterruptedException {
//        boolean got_at_least_one = false;
//        boolean got_two = false;
//        boolean tried_two = false;
//        double time_out = (isSide?20:19);
//        reset_prox();
//        for (int i=0; i<1; i++) {
//            // StraightIn(0.2,6);
//            got_at_least_one = autoIntakeOneGlyph(isSide, isBlue);
//        }
//        if(/*robot.runtimeAuto.seconds() < time_out-4 && got_at_least_one && !gotTwoGlyphs()*/ false) {
//            robot.snaked_left = false;
//            got_two = autoIntakeSecondGlyph(isSide, isBlue);
//        }
//        if(robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false) return got_at_least_one;
//        if (got_at_least_one && opModeIsActive()) {
//            //dumper_shake();
//            //intakeIn();
//            //sleep(100);
//            //intakeStop();
//            //dumper_shake();
//            robot.sv_dumper.setPosition(robot.SV_DUMPER_LIFT);
//        }
//        return got_at_least_one;
//    }

//    public boolean autoIntakeOneGlyph(boolean isSide, boolean isBlue) throws InterruptedException {
//        double time_out = (isSide?26:24.5);
//
//        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
//            return false;
//        }
//        boolean got_one = autoIntake(isSide, true, isBlue);
//        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
//            return false;
//        }
//
//        for (int i=0; (i<3) && !got_one; i++) { // try upto 3 times
//            if(robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false) return false;
//            got_one = autoIntake(isSide, true, isBlue);
//            if (!opModeIsActive()) return false;
//        }
//        return got_one;
//    }

//    public boolean autoIntakeSecondGlyph(boolean isSide,boolean isBlue) throws InterruptedException {
//        double time_out = (isSide?22:20);
//        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
//            return false;
//        }
//        StraightCm(0.4,15);
//        boolean got_two = autoIntake(isSide, false, isBlue);
//        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
//            return false;
//        }
//        for (int i=0; (i<1) && !got_two; i++) { // try upto 1 time
//            if(robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false) return false;
//            got_two = autoIntake(isSide, false, isBlue);
//            if (!opModeIsActive()) return false;
//        }
//
//        return got_two;
//    }

//    void intakeGateMid() {
//        if (!robot.use_intake || !robot.use_newbot_v2)
//            return;
//        robot.sv_intake_gate.setPosition(robot.SV_INTAKE_GATE_MID);
//    }

//    public boolean autoIntake(boolean isSide, boolean isFirstGlyph, boolean isBlue) throws InterruptedException {
//        double time_out = (isSide?26:24.5);
//        boolean got_one = false;
//        boolean curve_right = robot.snaked_left;
//        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > (time_out-1) && robot.servo_tune_up==false)) {
//            intakeStop(); stop_chassis();return false;
//        }
////        double cur_heading = imu_heading();
////        if (isBlue) { // drive to right
////            driveTT(-0.2,-0.1);
////        } else {
////            driveTT(-0.1,-0.2);
////        }
//        driveTTSnake(-0.5,(float) 1.0,curve_right);
//        robot.snaked_left = !robot.snaked_left;
//        intakeIn();
//        sleep(700);
//        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > (time_out-.5) && robot.servo_tune_up==false)) {
//            intakeStop(); stop_chassis();return false;
//        }
//        stop_chassis();
//        if(GlyphStuck()) {
//            if(!robot.tried_clockwise) {
//                correctGlyph(false);
//                robot.tried_clockwise = true;
//            }
//            else{
//                correctGlyph(false);
//                robot.tried_clockwise = false;
//            }
//        }
//        else{
//            intakeOut();
//            sleep(300);
//        }
//        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > time_out && robot.servo_tune_up==false)) {
//            intakeStop(); stop_chassis();return false;
//        }
//        //driveTTSnake(-0.3,(float) 1.0,!curve_right);
//        intakeIn();
//        sleep(600);
//        intakeStop();
//        stop_chassis();
//        if (!opModeIsActive() || (robot.runtimeAuto.seconds() > (time_out+0.5) && robot.servo_tune_up==false)) {
//            return false;
//        }
//        if(isFirstGlyph) {
//            got_one = gotOneGlyph();
//        }
//        else{
//            got_one = gotTwoGlyphs();
//        }
//        return got_one;
//    }

    public boolean GlyphStuck() {
        if (robot.rangeSensorBack==null)
            return false;
        return (getRange(RangeSensor.BACK)<5.1);
    }

//    boolean gotOneGlyph() {
//        boolean got_one=false;
//        if (robot.use_proximity_sensor) {
//            got_one = !robot.proxFL.getState() || !robot.proxML.getState();
//        }
//        return got_one;
//    }

//    boolean gotTwoGlyphs() {
//        boolean got_two=false;
//        if (robot.use_proximity_sensor) {
//            got_two = !robot.proxML.getState() && !robot.proxFL.getState();
//        }
//        return got_two;
//    }

}
