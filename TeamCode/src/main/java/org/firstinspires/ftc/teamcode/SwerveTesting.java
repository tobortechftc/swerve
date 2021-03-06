package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by Aurora on 12/3/17.
 */

@TeleOp(name = "TestSwerve", group = "Test")
public class SwerveTesting extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {
        robot.use_swerve = true;
        robot.use_imu = true;
        robot.use_encoder = true;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
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
        
        double desiredPower = .2;
        int desiredDistanceCm = 10;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            try{
                if (gamepad1.x) { // switch driving mode between CAR and CRAB
                    if (robot.cur_mode == SwerveDriveHardware.CarMode.CAR) {
                        change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
                    } else {
                        change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
                    }
                    sleep(500);
                }
                if(gamepad1.back && gamepad1.a){
                    if(!(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT)){// If in any other mode, switch to snake
                        change_swerve_pos(SwerveDriveHardware.CarMode.STRAIGHT);
                    }
                    else{ //Return from snake to previous drive mode
                        change_swerve_pos(robot.old_mode);
                    }
                    sleep(500);
                }
                if (gamepad1.dpad_up){
                    desiredDistanceCm += 10;
                    telemetry.addData("Set Distance", desiredDistanceCm).setRetained(true);
                    telemetry.addData("Set Power", desiredPower).setRetained(true);
                    telemetry.update();
                    sleep(500);
                }
                if (gamepad1.dpad_down){
                    desiredDistanceCm -= 10;
                    telemetry.addData("Set Distance", desiredDistanceCm).setRetained(true);
                    telemetry.addData("Set Power", desiredPower).setRetained(true);
                    telemetry.update();
                    sleep(500);
                }
                if (gamepad1.dpad_right){
                    desiredPower += 0.1;
                    telemetry.addData("Set Distance", desiredDistanceCm).setRetained(true);
                    telemetry.addData("Set Power", desiredPower).setRetained(true);
                    telemetry.update();
                    sleep(500);
                }
                if (gamepad1.dpad_left){
                    desiredPower -= 0.1;
                    telemetry.addData("Set Distance", desiredDistanceCm).setRetained(true);
                    telemetry.addData("Set Power", desiredPower).setRetained(true);
                    telemetry.update();
                    sleep(500);
                }

                if(gamepad1.a){
                    double startRangeDis = -1;
                    double endRangeDis = -1;

                    if(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT){
                        startRangeDis = getRange(RangeSensor.FRONT_RIGHT);
                    }
                    else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB){
                        startRangeDis = getRange(RangeSensor.FRONT_LEFT);
                    }

                    StraightCm(desiredPower, desiredDistanceCm);
                    sleep(1000);

                    if(robot.cur_mode == SwerveDriveHardware.CarMode.STRAIGHT){
                        endRangeDis = getRange(RangeSensor.FRONT_RIGHT);
                    }
                    else if(robot.cur_mode == SwerveDriveHardware.CarMode.CRAB){
                        endRangeDis = getRange(RangeSensor.FRONT_LEFT);
                    }

                    telemetry.addData("Range Start", startRangeDis).setRetained(true);
                    telemetry.addData("Range End", endRangeDis).setRetained(true);
                    telemetry.addData("Diff", endRangeDis - startRangeDis).setRetained(true);
                    telemetry.addData("Set Distance", desiredDistanceCm).setRetained(true);
                    telemetry.addData("Set Power", desiredPower).setRetained(true);
                    telemetry.update();
                }
                if(gamepad1.y){
                    StraightCm(-desiredPower, desiredDistanceCm);
                }

            } catch (Exception e){
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);
                telemetry.log().add(sw.toString());
                stop_chassis();
            }
        }
        stop_auto();
        stop_chassis();
    }
}
