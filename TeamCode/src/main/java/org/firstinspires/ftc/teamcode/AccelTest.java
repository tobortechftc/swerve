package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.io.PrintWriter;
import java.io.StringWriter;

/*
 * Created by Nick on 3/31/18.
 */


@Autonomous(name = "accel test", group = "NewBot")
public class AccelTest extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {

        robot.use_swerve = false;
        robot.use_arm = true;
        robot.use_imu = true;
        robot.use_encoder = true;
        robot.use_newbot = true;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
        robot.use_color_sensor = true;
        robot.use_Vuforia = false;
        robot.use_camera = false;
        robot.use_glyph_grabber = false;
        robot.use_proximity_sensor = true;
        robot.use_dumper = true;
        robot.use_intake = true;
        robot.use_relic_grabber = false;
        robot.use_relic_slider = false;
        robot.use_relic_grabber = false;
        robot.use_newbot_v2 = true;
        robot.use_imu2 = true;
        robot.allianceColor = TeamColor.BLUE;

        init_and_test();

        double highestX = 0;
        double highestY = 0;
        double highestZ = 0;
        double lowestX = 0;
        double lowestZ = 0;
        double lowestY = 0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        robot.runtime.reset();
        waitForStart();

        try {
            start_init();
        } catch (Exception e) {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            telemetry.log().add(sw.toString());
            sleep(15000);
            requestOpModeStop();
        }

        robot.runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            try {
                boolean boop = false;
                driveTT(.3, .3);
                sleep(500); // start for 0.5 sec to get stable speed before getting acceleratmeter value

                while (robot.runtime.seconds() <= 3 && boop==false) {
                    intakeGateMid();
                    boop = didBump();
                    if (robot.accel.xAccel > highestX) highestX = robot.accel.xAccel;
                    if (robot.accel.yAccel > highestY) highestY = robot.accel.yAccel;
                    if (robot.accel.zAccel > highestZ) highestZ = robot.accel.zAccel;
                    if (robot.accel.xAccel < lowestX) lowestX = robot.accel.xAccel;
                    if (robot.accel.yAccel < lowestY) lowestY = robot.accel.yAccel;
                    if (robot.accel.zAccel < lowestZ) lowestZ = robot.accel.zAccel;

                    telemetry.addData("xAccel/max/min=", "%.2f/%.2f/%.2f", robot.accel.xAccel,highestX,lowestX);
                    telemetry.addData("yAccel/max/min=", "%.2f/%.2f/%.2f", robot.accel.yAccel,highestY,lowestY);
                    telemetry.addData("zAccel/max/min=", "%.2f/%.2f/%.2f", robot.accel.zAccel,highestZ,lowestZ);
                    telemetry.update();
                }
                if (boop) StraightCm(.3, 20);
                else driveTT(0, 0);
                telemetry.addLine("Hit X button to exit the program...");
                telemetry.update();
                while (!gamepad1.x) {;}
            } catch (Exception e) {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);
                telemetry.log().add(sw.toString());
                stop_chassis();
            }
            stop_chassis();
        }
    }
}