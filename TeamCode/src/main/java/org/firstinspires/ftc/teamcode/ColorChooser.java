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

    ColorSensor colorSensor;

    final static int RED_BALL_MIN = -94;
    final static int RED_BALL_MAX = -36;
    final static int BLUE_BALL_MIN = 12;
    final static int BLUE_BALL_MAX = 66;
    String ballColor = "unknown";
    // Calculates difference between red and blue, and uses this to determine the ball's color
    public double calcDelta(ColorSensor co) {
        return (co.blue() - co.red());
    }

    boolean isRedBall(ColorSensor co) {
        if (calcDelta(co) >= RED_BALL_MIN && calcDelta(co) <= RED_BALL_MAX) {
            return true;
        }
        return false;
    }

    boolean isBlueBall(ColorSensor co) {
        if (calcDelta(co) >= BLUE_BALL_MIN && calcDelta(co) <= BLUE_BALL_MAX) {
            return true;
        }
        return false;
    }

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();


        while (opModeIsActive()) {

            // It saw a red ball
            if (isRedBall(colorSensor)) {
                ballColor = "red";
            }
            // It saw a blue ball
            else if (isBlueBall(colorSensor)) {
                ballColor = "blue";
            }
            else {
                ballColor = "unknown";
            }
            telemetry.addData("Time:", this.time);
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Delta ", colorSensor.red() - colorSensor.blue());
            telemetry.addData("The ball color is ", ballColor);
            telemetry.update();
            sleep(100);
        }
    }
}