package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

            doPlatformMission(true);
            //sleep(2000);
            telemetry.addData("Column", robot.targetColumn);
            telemetry.update();
            StraightIn(0.2, 22); // Drive off the balance stone
            //turnToColumn(robot.targetColumn, 0.2, true, true);
            go_to_distance_from(0.3, robot.targetColumn, true, true); // Drive to cryptobox.
            TurnLeftD(0.4, 90);
            StraightIn(0.5, 5);
            stop_chassis();
            //Deliver particle from the side
        }
    }
}
