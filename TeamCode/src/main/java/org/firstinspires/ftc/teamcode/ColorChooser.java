package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Nick on 9/16/2017.
 */
@TeleOp(name = "TestColor", group = "Test")
public class ColorChooser extends SwerveUtilLOP {

    TeamColor rightJewelColor = TeamColor.UNKNOWN;
    // Calculates difference between red and blue, and uses this to determine the ball's color
    //use to isolate the experiment for sensor


    @Override
    public void runOpMode() throws InterruptedException{
        robot.use_swerve = true;
        robot.use_imu = false;
        robot.use_Vuforia = true;
        robot.use_camera = true;
        robot.use_color_sensor = true;
        robot.use_arm = true;
        robot.use_glyph_grabber = true;
        robot.use_test_motor = false;
        robot.init(hardwareMap);
        robot.l_colorSensor.enableLed(true);
        robot.camera.activate();

        robot.allianceColor = TeamColor.BLUE;

        waitForStart();

        boolean start = false;
        while (opModeIsActive() && !start) {
            if (gamepad1.x) {
                if (robot.allianceColor==TeamColor.RED) robot.allianceColor = TeamColor.BLUE;
                else robot.allianceColor = TeamColor.RED;
                sleep(500);
            } else if (gamepad1.back) {
                start = true;
            }
            telemetry.addData("The current alliance is ", "%s", robot.allianceColor.toString());
            telemetry.update();
        }
        // It saw a red ball
        doPlatformMission(robot.allianceColor==TeamColor.BLUE);
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
            rightJewelColor = TeamColor.RED;
        } else if (robot.isBlueBall) {
            rightJewelColor = TeamColor.BLUE;
        } else {
            rightJewelColor = TeamColor.UNKNOWN;
        }

        telemetry.addData("Time:", this.time);
        telemetry.addData("Red  ", robot.red);
        telemetry.addData("Blue ", robot.blue);
        telemetry.addData("Delta ", (robot.blue - robot.red));
        telemetry.addData("The right ball color is ", "%s (%s side)", rightJewelColor.toString(), robot.allianceColor.toString());
        telemetry.update();
        sleep(10000);
        stop_auto();
    }
}