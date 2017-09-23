package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Nick on 9/16/2017.
 */
@TeleOp(name = "Swerve: ColorChooser", group = "Swerve")
public class ColorChooser extends LinearOpMode {

    ColorSensor colorSensor;
    Servo armYaw;   // side to side
    Servo armPitch; // up and down
    final static int RED_BALL_MIN = -89; // NOTE: DO RED - BLUE
    final static int RED_BALL_MAX = -31;
    final static int BLUE_BALL_MIN = 17;
    final static int BLUE_BALL_MAX = 61;
    final static double CHOOSE_RED = 1.0;
    final static double CHOOSE_BLUE = 0.0;


    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
//        armYaw = hardwareMap.servo.get("armYaw");


        waitForStart();

//        armPitch.setPosition(0.0);
        while (opModeIsActive()) {
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.update();
            sleep(100);
        }
    }
}
