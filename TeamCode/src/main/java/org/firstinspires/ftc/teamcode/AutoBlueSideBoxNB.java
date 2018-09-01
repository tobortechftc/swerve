package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.PrintWriter;
import java.io.StringWriter;

/*
 * Created by Mason on 2/15/2018.
 */


@Autonomous(name = "BlueSide", group = "NewBot2")
public class AutoBlueSideBoxNB extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {

        enable_hardware_for_auto();
        // set_verbose();  // uncomment this line to debug

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

                go_to_crypto(next_dist, .3, robot.targetColumn, true, true); // Drive to cryptobox
                deliverGlyph();
                robot.swerve.StraightTime(-.4,.5);
                robot.swerve.StraightCm(.4,20);
                robot.swerve.stop_chassis();
            } catch (Exception e) {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);
                telemetry.log().add(sw.toString());
                robot.swerve.stop_chassis();
            }
            robot.swerve.stop_chassis();
        }
    }
}