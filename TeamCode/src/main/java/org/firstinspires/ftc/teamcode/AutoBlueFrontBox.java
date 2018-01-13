package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by carlw on 11/4/2017.
 */

@Autonomous(name = "Swerve BlueFrontBox", group = "SwerveDrive")
public class AutoBlueFrontBox extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {

        robot.use_swerve = true;
        robot.use_imu = true;
        robot.use_encoder = true;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
        robot.use_color_sensor = true;
        robot.use_Vuforia = true;
        robot.use_camera = true;
        robot.use_arm = true;
        robot.use_glyph_grabber = true;
        robot.use_proximity_sensor = false;

        robot.allianceColor = TeamColor.BLUE;

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
                robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_CLOSED); // Closes to prevent range interference
                StraightIn(.2, 24); // Drive off the balance stone
                if (robot.use_proximity_sensor) StraightCm(.1, (getRange(RangeSensor.FRONT) - 35));
                alignUsingIMU();
                go_to_distance_from(0.3, get_cryptobox_column(), true, false, true); // Drive to cryptobox
                deliverGlyph();
                turnToCenter(true, false, robot.targetColumn);
                robot.sv_glyph_grabber_top.setPosition(robot.SV_GLYPH_GRABBER_TOP_OPEN);
                stop_chassis();
            }
            catch(Exception e) {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);

                telemetry.log().add(sw.toString());
                stop_chassis();
//                while(true) {
//                    sleep(1000);
//                }
            }
        }
    }
}
