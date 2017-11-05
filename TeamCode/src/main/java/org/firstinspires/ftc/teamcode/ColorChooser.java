package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Nick on 9/16/2017.
 */
@Autonomous(name = "Test-Blue", group = "Test")
public class ColorChooser extends SwerveUtilLOP {

    String ballColor = "unknown";
    // Calculates difference between red and blue, and uses this to determine the ball's color



    @Override
    public void runOpMode() throws InterruptedException{
        robot.use_swerve = false;
        robot.use_imu = false;
        robot.use_Vuforia = true;
        robot.use_camera = true;
        robot.use_color_sensor = true;
        robot.use_arm = true;
        robot.use_glyph_grabber = false;
        robot.use_test_motor = false;
        robot.init(hardwareMap);
        robot.colorSensor.enableLed(true);
        robot.camera.activate();

        //These constants are for setting a selected portion of the image from Camera
        //(Assuming portrait) Top left is (0,0), Top right (0,1), Bottom left is (1,0), Bottom right is (1,1)
        double IMAGE_WIDTH_CROP = 0.25;
        double IMAGE_HEIGHT_CROP = 0.33;
        double IMAGE_OFFSET_X = 0.75; // Cannot be 1, make sure take the respective crop into consideration
        double IMAGE_OFFSET_Y = 0; // Cannot be 1, make sure take the respective crop into consideration


        waitForStart();

        AllianceColor rightJewelColor = AllianceColor.UNKNOWN;

        if(robot.use_camera) {
            Bitmap bitmap = robot.camera.captureBitmap(IMAGE_OFFSET_X, IMAGE_OFFSET_Y, IMAGE_WIDTH_CROP, IMAGE_HEIGHT_CROP);
            AllianceColor leftJewelColor = AllianceColor.UNKNOWN;
            if (bitmap == null) {
                telemetry.addData("ERROR!", robot.camera.getLastError());
                telemetry.update();
                while (opModeIsActive());
                return;
            }
            leftJewelColor = determineJewelColor(bitmap);
            rightJewelColor = getOpposingColor(leftJewelColor);
            //Current mounting solution only allow camera to check the left jewel color
        }



        // It saw a red ball
        Jewel_Mission(true, rightJewelColor);
            /*
            if (gamepad2.a) {
                robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
            }
            if (gamepad2.y) {
                robot.sv_elbow.setPosition(robot.SV_ELBOW_UP);
            }
            if (gamepad2.x) {
                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_LEFT);
            }
            if (gamepad2.b) {
                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_RIGHT);
            }
            if (gamepad2.left_bumper) {
                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_DOWN);
            }
            */

        // checkBallColor();
        if (robot.isRedBall) {
            ballColor = "red";
        } else if (robot.isBlueBall) {
            ballColor = "blue";
        } else {
            ballColor = "unknown";
        }
        telemetry.addData("Time:", this.time);
        telemetry.addData("Red  ", robot.red);
        telemetry.addData("Blue ", robot.blue);
        telemetry.addData("Delta ", (robot.blue-robot.red));
        telemetry.addData("The ball color is ", ballColor);
        telemetry.update();
        sleep(5000);
        stop_auto();
    }
}