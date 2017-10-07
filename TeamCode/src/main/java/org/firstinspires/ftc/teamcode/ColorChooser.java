package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Nick on 9/16/2017.
 */
@TeleOp(name = "Swerve-ColorChooser", group = "Swerve")
public class ColorChooser extends SwerveUtilLOP {

    String ballColor = "unknown";
    // Calculates difference between red and blue, and uses this to determine the ball's color


    @Override
    public void runOpMode() throws InterruptedException{
        robot.use_swerve = false;
        robot.use_color_sensor = true;
        robot.use_Vuforia = false;
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            // It saw a red ball
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
    }
}