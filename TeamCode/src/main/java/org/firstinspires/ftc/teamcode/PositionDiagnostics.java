package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

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
                int pictograph = get_cryptobox_column();
                if (pictograph != -1){
                    telemetry.addData("Pictograph Visible!", pictograph);
                }

                double idealDistance = 39;
                double currentDistance = robot.rangeSensorLeft.getDistance(DistanceUnit.CM);

//                if (currentDistance + 2 > idealDistance || currentDistance - 2 < idealDistance) {
//
//                }
            }
        }
    }
