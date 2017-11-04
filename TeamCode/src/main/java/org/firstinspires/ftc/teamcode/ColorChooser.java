package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        robot.use_Vuforia = false;
        robot.use_color_sensor = true;
        robot.use_arm = true;
        robot.use_glyph_grabber = false;
        robot.use_test_motor = false;
        robot.init(hardwareMap);
        robot.colorSensor.enableLed(true);

        waitForStart();

        // It saw a red ball
        Jewel_Mission(true);
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