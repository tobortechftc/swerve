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
        robot.use_camera = false;
        robot.use_arm = true;


        int loops = 1;

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        robot.runtime.reset();
        waitForStart();


        doPlatformMission(true);
        //robot.targetColumn = get_cryptobox_column();
        //sleep(2000);

        telemetry.addData("Column", robot.targetColumn);
        telemetry.update();

        // run until the end of the match (driver presses STOP)

<<<<<<< Updated upstream
//        while (opModeIsActive()) {
//
//
//            if (loops == 1) {
//                StraightIn(0.5, 22); // Drive off the balance stone
//                turnToColumn(robot.targetColumn, 0.4, true, true);
//                //go_to_distance_from(0.3, robot.targetColumn, true); // Drive to cryptobox.
//                stop_chassis();
//                //Deliver particle from the side
//                loops++;
//            }
//        }
=======
            if (loops == 1) {
                StraightIn(0.2, 22); // Drive off the balance stone
                turnToColumn(robot.targetColumn, 0.2, true, true);
                //go_to_distance_from(0.3, 2, true); // Drive to cryptobox.
                stop_chassis();
                //Deliver particle from the side
                loops++;
            }
        }
>>>>>>> Stashed changes
    }
}
