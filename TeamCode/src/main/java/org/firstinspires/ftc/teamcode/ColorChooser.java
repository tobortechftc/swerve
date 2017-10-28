package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Nick on 9/16/2017.
 */
@TeleOp(name = "J-arm", group = "Test")
public class ColorChooser extends SwerveUtilLOP {

    String ballColor = "unknown";
    // Calculates difference between red and blue, and uses this to determine the ball's color

    public void Jewel_Mission() throws InterruptedException {
        arm_down();
        sleep(1000);
        checkBallColor();
        if (robot.isRedBall) {
            arm_right();
        }
        else if (robot.isBlueBall) {
            arm_left();
        }
        sleep(1000);
        arm_up();
    }

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

        waitForStart();

        while (opModeIsActive()) {
            // It saw a red ball
            if (gamepad2.right_bumper) {
                Jewel_Mission();
            }
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

            checkBallColor();
            if (robot.isRedBall) {
                ballColor = "red";
            } else if (robot.isBlueBall) {
                ballColor = "blue";
            } else {
                ballColor = "unknown";
            }
            telemetry.addData("Time:", this.time);
            telemetry.addData("Red  ", robot.colorSensor.red());
            telemetry.addData("Blue ", robot.colorSensor.blue());
            telemetry.addData("Delta ", calcDelta());
            telemetry.addData("The ball color is ", ballColor);
            telemetry.update();
            sleep(100);
        }
        stop_tobot();
    }
}