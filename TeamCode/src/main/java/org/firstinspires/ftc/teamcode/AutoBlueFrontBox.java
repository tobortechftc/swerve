package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
            doPlatformMission(true);
            StraightIn(0.2, 24); // Drive off the balance stone
            go_to_distance_from(0.3, get_cryptobox_column(), false); // Drive to cryptobox. Values are negative because driveTT goes backwards
            StraightIn(.2, 9); // Drives into cryptobox
            glyph_grabber_auto_open();
            StraightIn(-.2, 9);
        }
    }
}
