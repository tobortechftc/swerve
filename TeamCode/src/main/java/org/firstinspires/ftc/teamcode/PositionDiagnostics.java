package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Aurora on 11/9/17.
 */

@Autonomous(name = "Position Diagnostics", group = "SwerveDrive")
public class PositionDiagnostics extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {

            robot.use_swerve = false;
            robot.use_imu = false;
            robot.use_encoder = false;
            robot.use_minibot = false;
            robot.use_range_sensor = true;
            robot.use_color_sensor = false;
            robot.use_Vuforia = true;
            robot.use_camera = true;

            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Say", "Initialization Complete");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();



            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                if (robot.use_Vuforia) {
                    int pictograph = get_cryptobox_column();
                    if (pictograph != -1) {
                        telemetry.addData("Pictograph Visible!", pictograph);
                    } else {
                        telemetry.addData("Pictograph Not Visible!", null);
                    }
                }


                if (robot.use_range_sensor){
                double idealDistance = 39;
                double currentDistance = robot.rangeSensorLeft.getDistance(DistanceUnit.CM);
                double diff = currentDistance - idealDistance;

                    if (currentDistance + 2 > idealDistance || currentDistance - 2 < idealDistance) {
                        telemetry.addData("We are at a good Distance from the wall! Move", diff);
                    } else if (currentDistance + 2 < idealDistance) {
                        telemetry.addData("Robot is too close to the wall! Move", diff);
                    } else if (currentDistance - 2 > idealDistance) {
                        telemetry.addData("Robot is too far away from the wall! Move", diff);
                    } else {
                        telemetry.addData("You should not see this message, scream at Mason!", null);
                    }
                }


                if (robot.use_camera && robot.use_Vuforia){

                    //These constants are for setting a selected portion of the image from Camera
                    //(Assuming portrait) Top left is (0,0), Top right (0,1), Bottom left is (1,0), Bottom right is (1,1)
                    double IMAGE_WIDTH_CROP = 0.25;
                    double IMAGE_HEIGHT_CROP = 0.33;
                    double IMAGE_OFFSET_X = 0.75; // Cannot be 1, make sure take the respective crop into consideration
                    double IMAGE_OFFSET_Y = 0; // Cannot be 1, make sure take the respective crop into consideration

                    Bitmap bitmap = robot.camera.captureBitmap(IMAGE_OFFSET_X, IMAGE_OFFSET_Y, IMAGE_WIDTH_CROP, IMAGE_HEIGHT_CROP);
                    TeamColor jewel = determineJewelColor(bitmap);

                        //Current mounting solution only allows camera to check the left jewel color
                        if (jewel != TeamColor.UNKNOWN) {
                        telemetry.addData("I can see the Jewels!", jewel);
                    } else {
                        telemetry.addData("I can't see the Jewels!", null);
                    }
                }

                telemetry.update();
            }
        }
    }
