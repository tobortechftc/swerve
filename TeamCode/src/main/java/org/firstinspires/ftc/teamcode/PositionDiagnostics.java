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
                else {
                    telemetry.addData("Pictograph Not Visible!", null);
                }

                double idealDistance = 39;
                double currentDistance = robot.rangeSensorLeft.getDistance(DistanceUnit.CM);
                double diff = currentDistance - idealDistance;

//                if (currentDistance + 2 > idealDistance || currentDistance - 2 < idealDistance) {
//                    telemetry.addData("We are at a good Distance from the wall! Move", diff);
//                }
//                else if (currentDistance + 2 < idealDistance){
//                    telemetry.addData("Robot is too close to the wall! Move", diff);
//                }
//                else if (currentDistance -2 > idealDistance) {
//                    telemetry.addData("Robot is too far away from the wall! Move", diff);
//                }
//                else {
//                    telemetry.addData("You should not see this message, scream at Mason!", null);
//                }

                TeamColor jewel = checkBallColor();
                if (jewel != TeamColor.UNKNOWN){
                    telemetry.addData("I can see the Jewels!", jewel);
                }
                else {
                    telemetry.addData("I can't see the Jewels!", null);
                }
                telemetry.update();
            }
        }
    }
