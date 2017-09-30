package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Nick on 9/16/2017.
 */
@TeleOp(name = "Swerve: ColorChooser", group = "Swerve")
public class ColorChooser extends LinearOpMode {

//    HardwareMiniBot hw = new HardwareMiniBot();
//    HardwareMap hwMap = null;

    ColorSensor colorSensor;

    final static int RED_BALL_MIN = -94;
    final static int RED_BALL_MAX = -36;
    final static int BLUE_BALL_MIN = 12;
    final static int BLUE_BALL_MAX = 66;
    String ballColor = "unknown";
    // Calculates difference between red and blue, and uses this to determine the ball's color
    public double calcDelta() {
        return (colorSensor.blue() - colorSensor.red());
    }



    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "rev_co");

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Time:", this.time);
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Delta ", colorSensor.red() - colorSensor.blue());
            telemetry.addData("The ball color is ", ballColor);
            telemetry.update();
            sleep(100);

            // It saw a red ball
            if (calcDelta() >= RED_BALL_MIN && calcDelta() <= RED_BALL_MAX) {
                ballColor = "red";
            }
            // It saw a blue ball
            else if (calcDelta() >= BLUE_BALL_MIN && calcDelta() <= BLUE_BALL_MAX) {
                ballColor = "blue";
            }
            else {
                ballColor = "unknown";
            }
        }
    }
}