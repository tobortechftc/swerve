package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Aurora on 12/3/17.
 */

@TeleOp(name = "Swerve Testing", group = "SwerveDrive")
public class SwerveTesting extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {
        robot.use_swerve = true;
        robot.use_imu = true;
        robot.use_encoder = true;
        robot.use_minibot = false;
        robot.use_range_sensor = false;
        robot.use_color_sensor = false;
        robot.use_Vuforia = false;
        robot.use_glyph_grabber = true;
        robot.use_arm = true;

        init_and_test();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        start_init();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.back && gamepad1.a){
                if(!(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT)){// If in any other mode, switch to snake
                    change_swerve_pos(SwerveDriveHardware.CarMode.STRAIGHT);
                }
                else{ //Return from snake to previous drive mode
                    change_swerve_pos(robot.old_mode);
                }
                sleep(400);
            }

            if(gamepad1.a){
                double startRangeDis = robot.rangeSensorBack.getDistance(DistanceUnit.CM);
                StraightCm(.3, 60);
                double endRangeDis = robot.rangeSensorBack.getDistance(DistanceUnit.CM);

                telemetry.addData("Range Start", startRangeDis).setRetained(true);
                telemetry.addData("Range End", endRangeDis).setRetained(true);
                telemetry.addData("Diff", endRangeDis - startRangeDis).setRetained(true);
                telemetry.update();
            }
            if(gamepad1.y){
                StraightCm(-0.3, 60);
            }
        }
    }
}
