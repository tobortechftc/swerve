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
        AllianceColor rightJewelColor = AllianceColor.UNKNOWN;

        robot.use_swerve = true;
        robot.use_imu = true;
        robot.use_encoder = true;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
        robot.use_color_sensor = true;
        robot.use_Vuforia = true;

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Jewel_Mission(true, rightJewelColor);
            robot.targetColumn = get_cryptobox_column();
            StraightIn(0.5, 22); // Drive off the balance stone
            go_to_distance_from(0.5, robot.targetColumn, true);
            //Deliver particle from the side
        }
    }
}
