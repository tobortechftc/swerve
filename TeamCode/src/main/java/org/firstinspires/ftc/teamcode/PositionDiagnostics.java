package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by Aurora on 11/9/17.
 */

@TeleOp(name = "Position Diagnostics", group = "Test")
public class PositionDiagnostics extends SwerveUtilLOP {
    @Override
    public void runOpMode() throws InterruptedException {

//        robot.use_swerve = true;
//        robot.use_imu = true;
//        robot.use_encoder = false;
//        robot.use_minibot = false;
//        robot.use_range_sensor = true;
//        robot.use_color_sensor = false;
//        robot.use_glyph_grabber = false;
//        robot.use_Vuforia = true;
//        robot.use_camera = true;
//
//        robot.allianceColor = TeamColor.BLUE;
//
//        robot.init(hardwareMap, telemetry);
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Say", "Initialization Complete");    //
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//
//        // run until the end of the match (driver presses STOP)
//
//
//        while (opModeIsActive()) {
//            try {
//                if (gamepad1.back && gamepad1.a) {
//                    if (!(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT)) {// If in any other mode, switch to snake
//                        change_swerve_pos(SwerveDriveHardware.CarMode.STRAIGHT);
//                    } else { //Return from snake to previous drive mode
//                        change_swerve_pos(robot.old_mode);
//                    }
//                }
//                if (gamepad1.right_bumper && robot.allianceColor == TeamColor.BLUE) {
//                    robot.allianceColor = TeamColor.RED;
//                    sleep(300);
//                } else if (gamepad1.right_bumper && robot.allianceColor == TeamColor.RED) {
//                    robot.allianceColor = TeamColor.BLUE;
//                    sleep(300);
//                }
//                if (robot.use_Vuforia) {
//                    int pictograph = get_cryptobox_column();
//                    while (pictograph == -1) {
//                        telemetry.addData("Pictograph Not Visible!", null);
//                        sleep(200);
//                        pictograph = get_cryptobox_column();
//                        sleep(100);
//                    }
//                    if (pictograph != -1) {
//                        telemetry.addData("Pictograph visible", null);
//                    }
//                }
//
//                if (robot.use_range_sensor) {
//                    double idealDistance = 39;
//                    double currentDistance = 0;
//                    if (robot.allianceColor == TeamColor.BLUE) {
//                        currentDistance = getRange(RangeSensor.FRONT_LEFT);
//                    } else if (robot.allianceColor == TeamColor.RED) {
//                        currentDistance = getRange(RangeSensor.FRONT_RIGHT);
//                    } else {
//                        throw new IllegalArgumentException("allianceColor is not specified!");
//                    }
//
//                    double diff = currentDistance - idealDistance;
//
//                    if (currentDistance + 1 >= idealDistance && currentDistance - 1 <= idealDistance) {
//                        telemetry.addData("We are at a good Distance from the wall! Move", diff);
//                    } else if (currentDistance + 1 < idealDistance) {
//                        telemetry.addData("Robot is too close to the wall! Move", diff);
//                    } else if (currentDistance - 1 > idealDistance) {
//                        telemetry.addData("Robot is too far away from the wall! Move", diff);
//                    } else {
//                        telemetry.addData("You should not see this message, scream at Mason!", null);
//                    }
//                }
//
//                if (robot.use_imu) {
//                    double curHeading = imu_heading();
//                    telemetry.addData("IMU", curHeading);
//
//                    if (curHeading < 1 && curHeading > -1) {
//                        telemetry.addData("Heading is good!", curHeading);
//                    } else {
//                        telemetry.addData("Heading is off!", curHeading);
//                    }
//                }
//
//
////                    if (robot.use_camera && robot.use_Vuforia) {
////
////                        //These constants are for setting a selected portion of the image from Camera
////                        //(Assuming portrait) Top left is (0,0), Top right (0,1), Bottom left is (1,0), Bottom right is (1,1)
////                        double IMAGE_WIDTH_CROP = 1;
////                        double IMAGE_HEIGHT_CROP = 1;
////                        double IMAGE_OFFSET_X = 0.0; // Cannot be 1, make sure take the respective crop into consideration
////                        double IMAGE_OFFSET_Y = 0.0; // Cannot be 1, make sure take the respective crop into consideration
////
////                        ElapsedTime period = new ElapsedTime();
////
////                        Bitmap bitmap = robot.camera.captureBitmap(IMAGE_OFFSET_X, IMAGE_OFFSET_Y, IMAGE_WIDTH_CROP, IMAGE_HEIGHT_CROP);
////                        TeamColor jewel = determineJewelColor(bitmap);
////
////                        //Current mounting solution only allows camera to check the left jewel color
////                        if (jewel == TeamColor.UNKNOWN) {
////                            telemetry.addData("Camera can't see the Jewels!", jewel);
////                            bitmap = robot.camera.captureBitmap(IMAGE_OFFSET_X, IMAGE_OFFSET_Y, IMAGE_WIDTH_CROP, IMAGE_HEIGHT_CROP);
////                            jewel = determineJewelColor(bitmap);
////                        } else {
////                            telemetry.addData("Camera can see the Jewels!", null);
////                        }
////                    }
//
//                telemetry.update();
//            } catch (Exception e) {
//                StringWriter sw = new StringWriter();
//                PrintWriter pw = new PrintWriter(sw);
//                e.printStackTrace(pw);
//                telemetry.log().add(sw.toString());
//            }
//        }
//        stop_chassis();
//        stop_auto();
    }
}
