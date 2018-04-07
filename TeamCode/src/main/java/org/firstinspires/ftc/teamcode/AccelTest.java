package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.io.PrintWriter;
import java.io.StringWriter;

/*
 * Created by Nick on 3/31/18.
 */


@Autonomous(name = "accel test", group = "NewBot")
public class AccelTest extends SwerveUtilLOP{
    @Override
    public void runOpMode() throws InterruptedException {

        robot.use_swerve = false;
        robot.use_arm = true;
        robot.use_imu = true;
        robot.use_encoder = true;
        robot.use_newbot = true;
        robot.use_minibot = false;
        robot.use_range_sensor = true;
        robot.use_color_sensor = true;
        robot.use_Vuforia = false;
        robot.use_camera = false;
        robot.use_glyph_grabber = false;
        robot.use_proximity_sensor = true;
        robot.use_dumper = true;
        robot.use_intake = true;
        robot.use_relic_grabber = false;
        robot.use_relic_slider = false;
        robot.use_relic_grabber = false;
        robot.use_newbot_v2 = true;
        robot.use_imu2 = true;
        robot.allianceColor = TeamColor.BLUE;

        init_and_test();

        double highestX = 0;
        double highestY = 0;
        double highestZ = 0;
        double lowestX = 0;
        double lowestZ = 0;
        double lowestY = 0;
        double maxBump = 0;

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
        double speed = 0.2;
        while (opModeIsActive()) {
            telemetry.addData("0. X: start Y/A: +/- speed = ","%.2f", speed);
            telemetry.update();
            if (gamepad1.x)
                break;
            else if (gamepad1.y) {
                speed += 0.1;
                if (speed>1) speed=1.0;
                sleep(200);
            } else if (gamepad1.a) {
                speed -= 0.1;
                if (speed<-1) speed=-1.0;
                sleep(200);
            }
        }
        robot.runtime.reset();
        // robot.use_verbose = true;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            try {
                boolean boop = false;
                enable_bump_detection();
                StraightCm(-speed, 40);
                telemetry.addData("0.bumped=", "%s", (robot.bump_detected?"T":"F"));
                telemetry.addLine("7.Hit B/X button to go next/exit ...");
                telemetry.update();
                while (!gamepad1.x && !gamepad1.b) {;}
                if (gamepad1.x) break;
                robot.runtime.reset();
                driveTT(speed, speed);
                sleep(500); // start for 0.5 sec to get stable speed before getting acceleratmeter value
                double curPosFrontLeft = robot.motorFrontLeft.getCurrentPosition();
                double prev_lpos = curPosFrontLeft;
                double cur_time = robot.runtime.nanoseconds()/1000000000.0;
                double prev_time = cur_time;
                double cur_speed = 0, prev_speed=0;
                int iter = 0;
                while (robot.runtime.seconds() <= 3 && boop==false) {
                    //intakeGateMid();
                    //boop = didBump();
                    if (((++iter)%40)==0 && robot.stop_on_dump==true) {
                        cur_time = robot.runtime.nanoseconds()/1000000000.0;
                        curPosFrontLeft = robot.motorFrontLeft.getCurrentPosition();
                        cur_speed = (curPosFrontLeft-prev_lpos) / (cur_time-prev_time);
                        if (iter>120 && Math.abs(cur_speed-prev_speed)>100.0 && Math.abs(cur_speed)<0.25) {
                            robot.bump_detected = true;
                            boop = true;
                            break;
                        }
                        prev_lpos = curPosFrontLeft;
                        prev_time = cur_time;
                        prev_speed = cur_speed;
                    }
                    if (robot.accel.xAccel > highestX) highestX = robot.accel.xAccel;
                    if (robot.accel.yAccel > highestY) highestY = robot.accel.yAccel;
                    if (robot.accel.zAccel > highestZ) highestZ = robot.accel.zAccel;
                    if (robot.accel.xAccel < lowestX) lowestX = robot.accel.xAccel;
                    if (robot.accel.yAccel < lowestY) lowestY = robot.accel.yAccel;
                    if (robot.accel.zAccel < lowestZ) lowestZ = robot.accel.zAccel;
                    if (Math.abs(robot.accel.xAccel)+Math.abs(robot.accel.zAccel)>maxBump) {
                        maxBump = Math.abs(robot.accel.xAccel)+Math.abs(robot.accel.zAccel);
                    }

                    telemetry.addData("0.Max abs(x)+abs(z)=", "%.2f, bumped=%s", maxBump, (boop?"T":"F"));
                    //telemetry.addData("1.xAccel/max/min=", "%.2f/%.2f/%.2f", robot.accel.xAccel,highestX,lowestX);
                    //telemetry.addData("2.yAccel/max/min=", "%.2f/%.2f/%.2f", robot.accel.yAccel,highestY,lowestY);
                    //telemetry.addData("3.zAccel/max/min=", "%.2f/%.2f/%.2f", robot.accel.zAccel,highestZ,lowestZ);
                    telemetry.addData("4.Speed cur/prev=/i", "%.2f/%.2f/%1d", cur_speed,prev_speed,iter);
                    telemetry.addData("5.time cur/prev=", "%.4f/%.4f/", cur_time,prev_time);
                    telemetry.addData("6.enco cur/prev=", "%.2f/%.2f/", curPosFrontLeft,prev_lpos);
                    telemetry.update();
                }
                if (boop) StraightCm(speed, 20);
                else driveTT(0, 0);
                telemetry.addData("0.Max abs(x)+abs(z)=", "%.2f, bumped=%s", maxBump, (boop?"T":"F"));
                //telemetry.addData("1.xAccel/max/min=", "%.2f/%.2f/%.2f", robot.accel.xAccel,highestX,lowestX);
                //telemetry.addData("2.yAccel/max/min=", "%.2f/%.2f/%.2f", robot.accel.yAccel,highestY,lowestY);
                //telemetry.addData("3.zAccel/max/min=", "%.2f/%.2f/%.2f", robot.accel.zAccel,highestZ,lowestZ);
                telemetry.addData("4.Speed cur/prev/i=", "%.2f/%.2f/%1d", speed,prev_speed,iter);
                telemetry.addData("5.time cur/prev=", "%.4f/%.4f/", cur_time,prev_time);
                telemetry.addData("6.enco cur/prev=", "%.2f/%.2f/", curPosFrontLeft,prev_lpos);
                telemetry.addLine("7.Hit X button to exit the program...");
                telemetry.update();
                while (!gamepad1.x) {;}
            } catch (Exception e) {
                StringWriter sw = new StringWriter();
                PrintWriter pw = new PrintWriter(sw);
                e.printStackTrace(pw);
                telemetry.log().add(sw.toString());
                stop_chassis();
            }
            stop_chassis();
        }
        stop_tobot();
    }
}