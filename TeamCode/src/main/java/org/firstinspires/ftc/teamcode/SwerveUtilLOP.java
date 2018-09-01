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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.teamcode.hardware.SwerveSystem.CarMode.CAR;
import static org.firstinspires.ftc.teamcode.hardware.SwerveSystem.CarMode.CRAB;


public abstract class SwerveUtilLOP extends LinearOpMode {

    /* Declare OpMode members. */
      public SwerveDriveHardware robot           = new SwerveDriveHardware();

    /**
     * Is used for checking or determining a color based on an alliance
     * Can be used to set opposing or team alliance color
     */
    public enum TeamColor {
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

        if (robot.dumper.use_dumper) {
            robot.gg_slider_encoder_ok = true;
        }
    }

    public void start_init() throws InterruptedException {
        robot.runtimeAuto.reset();
        //if (robot.use_dumper) {
        //    lift_to_target(robot.LIFT_INIT_COUNT);
        //}
        if (robot.swerve.use_newbot_v2 && robot.jewel.use_arm && (robot.jewel.sv_jkicker!=null)) {
            robot.jewel.sv_jkicker.setPosition(robot.jewel.SV_JKICKER_UP);
        }
        if (robot.relicReachSystem.use_relic_grabber) {
            if (robot.swerve.use_newbot) {
                robot.relicReachSystem.relic_grabber_open(false);
            } else {
                robot.relicReachSystem.relic_grabber_open(false);
            }
            relic_arm_auto();
        }
        if (robot.intake.use_intake) {
            robot.intake.intakeGateUp();
        }
        if (robot.camera.use_Vuforia) {
            robot.targetColumn = robot.camera.get_cryptobox_column();
            telemetry.addData("0: Crypto Column =", robot.targetColumn);
            telemetry.update();
        }
        if (robot.swerve.use_imu) {
            if (robot.swerve.imu.getSystemStatus()== BNO055IMU.SystemStatus.SYSTEM_ERROR && robot.swerve.imu2!=null) {
                robot.swerve.use_imu2 = true;
            }
        }
        if (robot.use_verbose) {
            telemetry.addData("0: End start_init CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
            telemetry.update();
        }
    }

    public void set_verbose() {
        robot.use_verbose = true;
        robot.camera.use_verbose = true;
        robot.jewel.use_verbose = true;
        robot.intake.use_verbose = true;
        robot.dumper.use_verbose = true;
        robot.relicReachSystem.use_verbose = true;
        robot.swerve.use_verbose = true;
    }

    public void enable_hardware_for_auto() {
        robot.swerve.enable(true);
        robot.dumper.enable(true);
        robot.intake.enable(true);
        robot.camera.enable(true);
        robot.jewel.enable(true);
        robot.relicReachSystem.enable(true);
        robot.use_range_sensor = true;
        robot.use_proximity_sensor = true;
    }

    public void enable_hardware_for_teleop() {
        robot.swerve.enable(false);
        robot.dumper.enable(false);
        robot.intake.enable(false);
        robot.camera.disable();
        robot.jewel.enable(false);
        robot.relicReachSystem.enable(false);
        robot.use_range_sensor = false;
        robot.use_proximity_sensor = false;
    }

    /*
     **** [ 7. Autonomous or TeleOp Mission  ]
     */

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
            robot.targetColumn = robot.camera.get_cryptobox_column();
        }

        if (robot.swerve.use_newbot) {
            if (robot.allianceColor == TeamColor.BLUE || robot.swerve.use_newbot_v2) {
                robot.jewel.r_arm_down();
            } else {
                robot.jewel.l_arm_down();
            }
        } else { // oldBot
            if (robot.allianceColor == TeamColor.BLUE) {
                robot.jewel.l_arm_down();
            } else {
                robot.jewel.r_arm_down();
            }
        }

        if(robot.camera.use_camera) {
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
                        robot.camera.bitmap = robot.camera.captureBitmap(IMAGE_OFFSET_X, IMAGE_OFFSET_Y, IMAGE_WIDTH_CROP, IMAGE_HEIGHT_CROP);
                    }
                    while (runTime.seconds() < 0.5 && robot.camera.bitmap == null);

                    if (robot.camera.bitmap != null) {
                        robot.camera.leftJewelColorCamera = determineJewelColor(robot.camera.bitmap);
                        telemetry.addData("Left Jewel Color", robot.camera.leftJewelColorCamera);
                        robot.camera.rightJewelColorCamera = robot.camera.leftJewelColorCamera.getOpposingColor();
                        robot.camera.camReady = true;
                        robot.camera.stopCamera();
                    }
                }
            }).start();
        }

        if (!opModeIsActive()) return 0;

        if (robot.swerve.use_newbot_v2)
            robot.intake.intakeGateUp();

        if (!opModeIsActive()) return 0;

        sleep(500);
        if (!opModeIsActive()) return 0;

        TeamColor rightJewelColorCS = robot.jewel.checkBallColor(isBlueAlliance);

        // Comparing of sensor data
        boolean isRedBall = (rightJewelColorCS == TeamColor.RED);
        boolean isBlueBall = (rightJewelColorCS == TeamColor.BLUE);

        telemetry.addData("isBlueBall/isRedBall", "%s/%s", isBlueBall, isRedBall);
        telemetry.update();
        // sleep(2000);
        double next_dist = 0;

        //gives time for icamera if thread isn't done
        if (!robot.camera.camReady) {
            telemetry.addData("Camera wasn't ready on time!", null).setRetained(true);
            sleep(100);
        }

        // Determines if right jewel is red
        int directionI = calcArmDirection(rightJewelColorCS, robot.camera.rightJewelColorCamera, isBlueAlliance);
        if (robot.swerve.use_newbot_v2 && !isBlueAlliance) {
            directionI *= -1;
        }
        if (!opModeIsActive()) return 0;
        if (robot.jewel.sv_jkicker!=null) {
            if (directionI<0)
                robot.jewel.jkick_left();
            else if (directionI>0)
                robot.jewel.jkick_right();
            robot.jewel.r_arm_up();
        } else if (robot.swerve.use_newbot) {
            int dist = (directionI > 0 ? 7 : 6);
            robot.swerve.StraightCm(-.2 * directionI, dist); // Drives forward if right jewel is red, backwards if blue
            sleep(100);
            if (isBlueAlliance || robot.swerve.use_newbot_v2)
                robot.jewel.r_arm_up(); // arm up to ensure the jewel is knocked out
            else
                robot.jewel.l_arm_up(); // arm up to ensure the jewel is knocked out
            // next_dist = -1.0*dist*directionI;
            robot.swerve.StraightCm(.4 * directionI, dist); // Drives forward if right jewel is blue, backwards if red
        } else { // oldBot
            if (isBlueAlliance) {
                if (directionI == 1) { // Right jewel is our color
                    robot.jewel.arm_left();
                } else if (directionI == -1) { // Right jewel is their color
                    robot.jewel.arm_right();
                }
                // if directionI 0, then color is unknown
            } else {
                int dist = (directionI > 0 ? 7 : 5);
                robot.swerve.StraightCm(.1 * directionI, dist); // Drives forward if right jewel is red, backwards if blue
                sleep(100);
                robot.jewel.r_arm_up(); // arm up to ensure the jewel is knocked out
                robot.swerve.StraightCm(-.15 * directionI, dist); // Drives forward if right jewel is blue, backwards if red
            }
        }
        if (!opModeIsActive()) return next_dist;

        if (isBlueAlliance && !robot.swerve.use_newbot) {
            sleep(500);
            robot.jewel.l_arm_up();
        }
        if (robot.use_verbose)
            telemetry.addData("0: End doPlatformMission CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());

        return next_dist;
    }

    // Provided two colors guesses and team color, determines if the right ball is the same color as our team.
    // if sum = 1, then it is our color. if sum = 0, then it is unknown. if sum = -1, then it is their color.
    // Ex. sensor is red, icamera is unknown, team is red. It will return 1.
    public int calcArmDirection(TeamColor sensor, TeamColor camera, boolean isBlueAlliance) {
        int result = 0;
        if (sensor == TeamColor.RED)
            result = 1;
        else if (sensor == TeamColor.BLUE)
            result = -1;
        else { // Sensor trumps icamera, only use icamera when sensor is unknown
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
            robot.dumper.dumper_vertical();
            // dumper_up();
            if (!opModeIsActive()) return;
            sleep(500);
            if (!opModeIsActive()) return;
            robot.swerve.driveTT(0.4, 0.4); // drive backward for .3 sec
            sleep(250);
            robot.swerve.driveTT(0,0);
            if (!opModeIsActive()) return;
            robot.swerve.StraightIn(0.8, 6);
            robot.dumper.dumper_down(true);
        }
        if (robot.use_verbose)
            telemetry.addData("0: End deliveryGlyph() CPU time =", "%3.2f sec", robot.runtimeAuto.seconds());
    }

//    TeamColor checkBallColor(boolean isBlueAlliance) throws InterruptedException {
//        boolean isBlueBall = false;
//        boolean isRedBall = false;
//        robot.runtime.reset();
//        double d = calcDelta(isBlueAlliance);
//        TeamColor result = TeamColor.UNKNOWN;
//
//        while (!isBlueBall && !isRedBall && (robot.runtime.seconds()<1.5)) {
//            if ((d >= robot.BLUE_BALL_MIN) && (d <= robot.BLUE_BALL_MAX)) {
//                isBlueBall = true;
//            } else {
//                isBlueBall = false;
//            }
//            if (d >= robot.RED_BALL_MIN && d <= robot.RED_BALL_MAX) {
//                isRedBall = true;
//            } else {
//                isRedBall = false;
//            }
//
//            d = calcDelta(isBlueAlliance);
//        }
////        telemetry.addData("delta/isBlueBall/isRedBall=", "%3.1f/%s/%s",d,robot.isBlueBall,robot.isRedBall);
////        telemetry.update();
//        if (isBlueBall && isRedBall) {
//            result = TeamColor.UNKNOWN;
//        } else if (isBlueBall) {
//            result = TeamColor.BLUE;
//        }
//        else if (isRedBall) {
//            result = TeamColor.RED;
//        }
//        else {
//            result = TeamColor.UNKNOWN;
//        }
//        return result;
//    }

//    int getColumnIndex(RelicRecoveryVuMark vuMark) throws InterruptedException {
//        // return row index for Cryptograph
//        // unknown : -1
//        // left    :  0
//        // center  :  1
//        // right   :  2
//        if (vuMark == RelicRecoveryVuMark.LEFT)
//            return 0;
//        else if (vuMark == RelicRecoveryVuMark.CENTER)
//            return 1;
//        else if (vuMark == RelicRecoveryVuMark.RIGHT)
//            return 2;
//
//        return -1;
//    }

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
        if (robot.jewel!=null)
            robot.jewel.stop();

        if (robot.intake.sv_bar_wheel!=null) {
            robot.intake.sv_bar_wheel.close();
        }
        // stop all sensors
    }



//    void intakeTurn(boolean clockwise) {
//        if (!robot.use_intake)
//            return;
//        if (clockwise) {
//            robot.mt_intake_left.setPower(-robot.intake.intakeRatio / 2.0);
//            robot.mt_intake_right.setPower(robot.intake.intakeRatio);
//        } else {
//            robot.mt_intake_left.setPower(robot.intake.intakeRatio);
//            robot.mt_intake_right.setPower(-robot.intake.intakeRatio / 2);
//        }
//    }
//
//    void intakeOut() {
//        if (!robot.use_intake)
//            return;
//        robot.mt_intake_left.setPower(-1.0*robot.intake.intakeRatio);
//        robot.mt_intake_right.setPower(-1.0*robot.intake.intakeRatio);
//        robot.intake.intakeBarWheelOut();
//    }
//
//    void intakeStop() {
//        if (!robot.use_intake)
//            return;
//        robot.mt_intake_left.setPower(0);
//        robot.mt_intake_right.setPower(0);
//        robot.intake.intakeBarWheelStop();
//    }

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
        telemetry.addData("1. Team", " %s sw/IMU/Vu = %s%s/%s/%s",
                robot.allianceColor, (robot.swerve.use_swerve||robot.swerve.use_newbot ?"Y":"N"),
                (robot.isTesting?"-T":""), (robot.swerve.use_imu?"Y":"N"),
                (robot.camera.use_Vuforia ?"Y":"N"));
        telemetry.addData("2. PW-r/L/R/Sl-En =", "%.2f,%.2f/%.2f/%s",
                robot.swerve.drivePowerRatio, robot.swerve.motorPowerLeft,robot.swerve.motorPowerRight,(robot.gg_slider_encoder_ok ?"Y":"N"));
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
        if (robot.jewel.use_color_sensor) {
            telemetry.addData("5.2 color =", "R b/r = %3d/%3d",
                    robot.jewel.r_colorSensor.blue(), robot.jewel.r_colorSensor.red());
        }

        if (robot.camera.use_Vuforia) {
            telemetry.addData("5. Vuforia Column = ", "%d", robot.camera.get_cryptobox_column());
        }
        if ((robot.swerve.use_swerve || robot.swerve.use_newbot)&& !robot.servo_tune_up) {
            telemetry.addData("6. Drive Mode = ", "%s", robot.swerve.cur_mode);
            telemetry.addData("6.1 SW-Enc FL/FR/BL/BR = ", "%d/%d/%d/%d",
                    robot.swerve.motorFrontLeft.getCurrentPosition(),
                    robot.swerve.motorFrontRight.getCurrentPosition(),
                    (robot.swerve.motorBackLeft!=null?robot.swerve.motorBackLeft.getCurrentPosition():0),
                    (robot.swerve.motorBackRight!=null?robot.swerve.motorBackRight.getCurrentPosition():0));
        }
        if (robot.intake.use_intake) {
            telemetry.addData("7. Intake r = ", "%2.2f", robot.intake.intakeRatio);
        }

        if (robot.dumper.use_dumper)  {
            telemetry.addData("8.1 dumper slide / pos = ","%d (pw=%.2f)/%3.3f",
                    robot.dumper.mt_lift.getCurrentPosition(), robot.dumper.mt_lift.getPower(),
                    robot.dumper.sv_dumper.getPosition());
        }

        if (robot.relicReachSystem.use_relic_grabber) {
            telemetry.addData("9.1 relic gr/wrist/elbow = ", "%4.4f/%4.3f/%4.3f",
                    robot.relicReachSystem.sv_relic_grabber.getPosition(), robot.relicReachSystem.sv_relic_wrist.getPosition(),
                    (robot.relicReachSystem.sv_relic_elbow!=null?robot.relicReachSystem.sv_relic_elbow.getPosition():0));
        }
        if (robot.relicReachSystem.use_relic_slider) {
            telemetry.addData("9.2 r-slider pwr/enc = ","%3.2f/%d",
                    robot.relicReachSystem.mt_relic_slider.getPower(),robot.relicReachSystem.mt_relic_slider.getCurrentPosition());
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
