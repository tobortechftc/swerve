package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by carlw on 11/4/2017.
 */
@Disabled
@Autonomous(name = "RedSideBoxProximity", group = "Proximity")
public class AutoRedSideBoxProx extends SwerveUtilLOP{
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
        robot.use_proximity_sensor = true;

        robot.allianceColor = TeamColor.RED;

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
                doPlatformMission(false);
                //telemetry.addData("Column", robot.targetColumn);
                //telemetry.update();
                go_to_crypto_prox(0.3, robot.targetColumn, false, true); // Drive to cryptobox.
                deliverGlyph();
                turnToCenter(false, true, robot.targetColumn);
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
