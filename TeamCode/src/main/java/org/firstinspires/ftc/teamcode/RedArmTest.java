package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Nick on 9/16/2017.
 */
@Disabled
@Autonomous(name = "Test-Red", group = "Test")
public class RedArmTest extends SwerveUtilLOP {

    TeamColor rightJewelColor = TeamColor.UNKNOWN;
    // Calculates difference between red and blue, and uses this to determine the ball's color
    //use to isolate the experiment for sensor


    @Override
    public void runOpMode() throws InterruptedException{
//        robot.use_swerve = false;
//        robot.use_imu = false;
//        robot.use_Vuforia = true;
//        robot.use_camera = true;
//        robot.use_color_sensor = true;
//        robot.use_arm = true;
//        robot.use_glyph_grabber = false;
//        robot.use_test_motor = false;
//        robot.init(hardwareMap, telemetry);
//        robot.l_colorSensor.enableLed(true);
//        robot.camera.activate();
//
//
//        waitForStart();
//
//
//        // It saw a red ball
//        doPlatformMission(true);
//            /*
//            if (gamepad2.a) {
//                robot.sv_elbow.setPosition(robot.SV_ELBOW_DOWN);
//            }
//            if (gamepad2.y) {
//                robot.sv_elbow.setPosition(robot.SV_ELBOW_UP);
//            }
//            if (gamepad2.x) {
//                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_LEFT);
//            }
//            if (gamepad2.b) {
//                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_RIGHT);
//            }
//            if (gamepad2.left_bumper) {
//                robot.sv_shoulder.setPosition(robot.SV_SHOULDER_DOWN);
//            }
//            */
//
//        // checkBallColor();
//        if (robot.isRedBall) {
//            rightJewelColor = TeamColor.RED;
//        } else if (robot.isBlueBall) {
//            rightJewelColor = TeamColor.BLUE;
//        } else {
//            rightJewelColor = TeamColor.UNKNOWN;
//        }
//
//
//
//        telemetry.addData("Time:", this.time);
//        telemetry.addData("Red  ", robot.red);
//        telemetry.addData("Blue ", robot.blue);
//        telemetry.addData("Delta ", (robot.blue-robot.red));
//        telemetry.addData("The right ball color is ", rightJewelColor);
//        telemetry.update();
//        sleep(5000);
//        stop_auto();
    }
}