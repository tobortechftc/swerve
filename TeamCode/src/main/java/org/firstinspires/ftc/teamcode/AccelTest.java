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
        robot.use_Vuforia = true;
        robot.use_camera = true;
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
                driveTT(.3, .3);
                boolean boop = false;
                while (robot.runtime.seconds() <= 3 && boop==false) {
                    intakeGateMid();
                    boop = didBump();
                    if (robot.accel.xAccel > highestX) highestX = robot.accel.xAccel;
                    if (robot.accel.yAccel > highestY) highestY = robot.accel.yAccel;
                    if (robot.accel.zAccel > highestZ) highestZ = robot.accel.zAccel;
                    if (robot.accel.xAccel < lowestX) lowestX = robot.accel.xAccel;
                    if (robot.accel.yAccel < lowestY) lowestY = robot.accel.yAccel;
                    if (robot.accel.zAccel < lowestZ) lowestZ = robot.accel.zAccel;

                    telemetry.addData("xAccel", String.format("%.5f", robot.accel.xAccel));
                    telemetry.addData("yAccel", String.format("%.5f", robot.accel.yAccel));
                    telemetry.addData("zAccel", String.format("%.5f", robot.accel.zAccel));
                    telemetry.addLine();
                    telemetry.addData("HighestX", String.format("%.5f", highestX));
                    telemetry.addData("LowestX", String.format("%.5f", lowestX));
                    telemetry.addLine();
                    telemetry.addData("HighestY", String.format("%.5f", highestY));
                    telemetry.addData("LowestY", String.format("%.5f", lowestY));
                    telemetry.addLine();
                    telemetry.addData("HighestZ", String.format("%.5f", highestZ));
                    telemetry.addData("LowestZ", String.format("%.5f", lowestZ));
                    telemetry.update();
                }
                if (boop) StraightCm(.3, 20);
                else driveTT(0, 0);
                sleep(10000);
            } catch (Exception e) {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);
                telemetry.log().add(sw.toString());
                stop_chassis();
//                while (true) {
//                    sleep(1000);
//                }
            }
            stop_chassis();
        }
    }
}