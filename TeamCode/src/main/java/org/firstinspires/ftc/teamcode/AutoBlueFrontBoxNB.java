package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.PrintWriter;
import java.io.StringWriter;

/*
 * Created by Mason on 2/15/2018.
 */


@Autonomous(name = "BlueFront", group = "NewBot2")
public class AutoBlueFrontBoxNB extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {

        robot.swerve.use_swerve = false;
        robot.use_arm = true;
        robot.swerve.use_imu = true;
        robot.swerve.use_encoder = true;
        robot.swerve.use_newbot = true;
        robot.use_range_sensor = true;
        robot.use_color_sensor = true;
        robot.use_Vuforia = true;
        robot.use_camera = true;
        robot.use_glyph_grabber = false;
        robot.use_proximity_sensor = true;
        robot.use_dumper = true;
        robot.relicReachSystem.use_relic_grabber = false;
        robot.relicReachSystem.use_relic_slider = false;
        robot.relicReachSystem.use_relic_grabber = false;
        robot.swerve.use_newbot_v2 = true;

        robot.allianceColor = TeamColor.BLUE;

        init_and_test();

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


        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            try {
                double next_dist = doPlatformMission(true);
                go_to_crypto(next_dist, .3, robot.targetColumn, true, false); // Drive to cryptobox
                deliverGlyph();
                robot.swerve.StraightTime(-.4,.5);
                robot.swerve.StraightCm(.4,20);
                //turnToCenter(true, false, robot.targetColumn);
                robot.swerve.stop_chassis();
            } catch (Exception e) {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);
                telemetry.log().add(sw.toString());
                robot.swerve.stop_chassis();
//                while (true) {
//                    sleep(1000);
//                }
            }
            robot.swerve.stop_chassis();
        }
    }
}