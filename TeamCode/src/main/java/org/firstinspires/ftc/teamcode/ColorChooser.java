package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Nick on 9/16/2017.
 */
@Autonomous(name = "Swerve: ColorChooser", group = "Swerve")
public class ColorChooser extends LinearOpMode {

    ColorSensor colorSensor;
    Servo armYaw;
    Servo armPitch;
    final static double RED = 6.0;  // These values are just placeholders
    final static double BLUE = 4.0;
    final static double CHOOSE_RED = 1.0;
    final static double CHOOSE_BLUE = 0.0;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        armYaw = hardwareMap.servo.get("armAngle");

        armPitch.setPosition(0.0);
        while (opModeIsActive()) {
            if (colorSensor.red() >= RED) { // The color sensor sees red and angles arm to hit red jewel
                armYaw.setPosition(CHOOSE_RED);
            }
            else if (colorSensor.blue() >= BLUE) { // The color sensor sees blue and angles arm to hit blue jewel
                armYaw.setPosition(CHOOSE_BLUE);
            }
            else { // The color sensor didn't see red nor blue
                // well crap
            }
        }
    }
}
