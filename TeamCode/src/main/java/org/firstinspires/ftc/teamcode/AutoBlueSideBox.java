package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by carlw on 11/4/2017.
 */

@Autonomous(name = "Swerve BlueSideBox", group = "SwerveDrive")
public class AutoBlueSideBox extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {

        robot.use_swerve = true;
        robot.use_arm = true;
        robot.use_imu = true;
        robot.use_encoder = true;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
        robot.use_color_sensor = true;
        robot.use_Vuforia = true;
        robot.use_camera = true;
        robot.use_glyph_grabber = true;
        robot.use_arm = true;

        robot.allianceColor = TeamColor.BLUE;


        int loops = 1;

        init_and_test();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        robot.runtime.reset();
        waitForStart();

        start_init();


        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            try {
                doPlatformMission(true);
                //sleep(2000);
                telemetry.addData("Column", robot.targetColumn);
                telemetry.update();
                StraightIn(0.2, 22); // Drive off the balance stone
                alignUsingIMU();
                sleep(500);

                //turnToColumn(robot.targetColumn, 0.2, true, true);
                go_to_distance_from(0.3, robot.targetColumn, true, true, true); // Drive to cryptobox.
                deliverGlyph();
                turnToCenter(true,true,robot.targetColumn);
                stop_chassis();
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
