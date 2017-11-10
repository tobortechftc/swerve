package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Aurora on 11/9/17.
 */

@Autonomous(name = "Position Diagnostics", group = "SwerveDrive")
public class PositionDiagnostics extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {

            robot.use_swerve = false;
            robot.use_imu = false;
            robot.use_encoder = false;
            robot.use_minibot = false;
            robot.use_range_sensor = true;
            robot.use_color_sensor = false;
            robot.use_Vuforia = true;
            robot.use_camera = true;

            robot.init(hardwareMap);

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Say", "Initialization Complete");    //
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                
            }
        }
    }
