package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by Nick on 12/8/2017.
 */

@Autonomous(name = "RedFront-NB-V2-PLUS", group = "NewBot")
public class AutoRedFrontBoxNBV2Plus extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {

        robot.use_swerve = false;
        robot.use_arm = true;
        robot.use_imu = true;
        robot.use_encoder = true;
        robot.use_newbot = true;
        robot.use_newbot_v2 = true;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
        robot.use_color_sensor = true;
        robot.use_Vuforia = true;
        robot.use_camera = true;
        robot.use_glyph_grabber = false;
        robot.use_proximity_sensor = true;
        robot.use_dumper = true;
        robot.use_intake = true;
        robot.use_relic_grabber = false;
        robot.use_relic_slider = false;

        robot.allianceColor = TeamColor.RED;

        init_and_test();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        robot.runtime.reset();
        waitForStart();

        try {
        start_init();
        } catch (Exception e) {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            telemetry.log().add(sw.toString());
            sleep(15000);
            requestOpModeStop();
        }


        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            try {
                double next_dist = doPlatformMission(false);
                go_to_crypto_prox_NB(next_dist, .3, robot.targetColumn, false, false); // Drive to cryptobox
                deliverGlyph();
                if(robot.targetColumn > 0) {
                    change_swerve_pos(SwerveDriveHardware.CarMode.CRAB);
                    sleep(200);
                    StraightCm(0.5, 18.5 * robot.targetColumn);
                    change_swerve_pos(SwerveDriveHardware.CarMode.CAR);
                }
                alignUsingIMU(170);
                grabAndDump(false);


                stop_chassis();
            } catch (Exception e) {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);
                telemetry.log().add(sw.toString());
                stop_chassis();
//                while (true) {
//                    sleep(1000);
//                }
            }
            stop_chassis();
        }
    }
}
