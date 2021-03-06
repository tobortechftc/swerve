package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by Aurora on 12/10/17.
 */
    @Disabled
    @Autonomous(name = "Platform Testing", group = "SwerveDrive")
    public class PlatformTesting extends SwerveUtilLOP{
        @Override
        public void runOpMode() throws InterruptedException {

            robot.use_swerve = true;
            robot.use_arm = true;
            robot.use_imu = true;
            robot.use_encoder = true;
            robot.use_minibot = false;
            robot.use_range_sensor = true;
            robot.use_color_sensor = false;
            robot.use_Vuforia = true;
            robot.use_camera = true;
            robot.use_glyph_grabber = true;
            robot.use_arm = true;

            robot.allianceColor = TeamColor.BLUE;

//            robot.init(hardwareMap);


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
                } catch (Exception e) {
                    StringWriter sw = new StringWriter();
                    PrintWriter pw = new PrintWriter(sw);
                    e.printStackTrace(pw);
                    telemetry.log().add(sw.toString());
                    stop_chassis();
                while (true) {
                    sleep(1000);
                }
                }
                stop_chassis();
            }
        }
    }