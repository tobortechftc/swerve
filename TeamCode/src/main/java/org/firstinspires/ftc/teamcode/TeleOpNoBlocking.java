package org.firstinspires.ftc.teamcode;

/**
 * Created by carlw on 9/1/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TeleOp-NoBlock", group="Test-NewBot")
public class TeleOpNoBlocking extends SwerveUtilLOP{
        /* Declare OpMode members. */
    //SwerveDriveHardware robot           = new SwerveDriveHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        int adj_count = 0; // count to smooth out car turn

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.use_swerve = false;
        robot.use_newbot = true;
        robot.use_newbot_v2 = true;
        robot.use_front_drive_only = false; // front drive only
        robot.use_intake = true;
        robot.use_dumper = true;
        robot.use_dumper_gate = true;
        robot.use_imu = false;
        robot.use_encoder = true;
        robot.use_minibot = false;
        robot.use_range_sensor = false;
        robot.use_color_sensor = false;
        robot.use_proximity_sensor = false;
        robot.use_Vuforia = false;
        robot.use_glyph_grabber = false;
        robot.use_relic_grabber = true;
        robot.use_relic_slider = true;
        robot.use_relic_elbow = false;
        robot.use_arm = true;
        robot.use_front_arm = false;

        init_and_test();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Initialization Complete");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        start_init();
        relic_arm_up(); // arm up to prevent intake gate collision
        while (opModeIsActive()) {
            checkGamepadOneInputs(false);
            checkGamepadTwoInputs(false);
            show_telemetry();
        }
        stop_tobot();
    }
}
