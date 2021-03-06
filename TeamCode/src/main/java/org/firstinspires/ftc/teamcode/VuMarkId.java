/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.provider.MediaStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.io.FileOutputStream;
import java.io.OutputStream;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */
@Disabled
@Autonomous(name="Vu-Test", group ="Test")
public class VuMarkId extends SwerveUtilLOP {

    //Constants:
    //(Assuming portrait) Top left is (0,0), Top right (0,1), Bottom left is (1,0), Bottom right is (1,1)
    double IMAGE_WIDTH_CROP = 1;
    double IMAGE_HEIGHT_CROP = 1;
    double IMAGE_OFFSET_X = 0; // Cannot be 1, make sure take the respective crop into consideration
    double IMAGE_OFFSET_Y = 0; // Cannot be 1, make sure take the respective crop into consideration

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        int box_column = -1;
        robot.use_swerve = false;
        robot.use_imu = false;
        robot.use_color_sensor = false;
        robot.use_range_sensor = false;
        robot.use_arm = false;
        robot.use_glyph_grabber = false;
        robot.use_relic_grabber = false;
        robot.use_Vuforia = true;
        robot.use_camera = true;

        robot.allianceColor = TeamColor.BLUE;

        robot.init(hardwareMap,telemetry);

        waitForStart();

        if (robot.use_Vuforia) {
            box_column = get_cryptobox_column();
        }

        int state = 0;
        double stepStart = this.time;
        int LoopRep = 0;
        Bitmap bitmap = null;

        /* The state variable reflects what the loop is attempting to do:
            -1 = quiting op mode (frame==null)
            0 = awaiting activation
            1 = attempting to get frame queue
            2 = Frame retrieved
         */
        while (opModeIsActive() && (runtime.seconds() < 29)) {
            LoopRep++;
            //show_telemetry();
            robot.waitForTick(40);
            telemetry.addData("Box column = ", box_column);
            if (state == 0) {
                int capacity = robot.vuforia.getFrameQueueCapacity();
                telemetry.addData("6. Vuforia Queue State = ", capacity);
                if (this.time - stepStart > 0) {
                    state = 1;
                    stepStart = this.time;
                    robot.camera.activate();
                }
            } else if (state == 1) {
                bitmap = robot.camera.captureBitmap(IMAGE_OFFSET_X, IMAGE_OFFSET_Y, IMAGE_WIDTH_CROP, IMAGE_HEIGHT_CROP);

                if (bitmap == null) {
                    telemetry.addData("Couldn't get a bitmap", robot.camera.getLastError());
                    if (this.time - stepStart > 0.5) {
                        state = -1;
                        continue;
                    }
                }
                if (bitmap != null) {
                    //Save

                    //int whitestPixel = robot.camera.getWhitestPixel(bitmap);
                    //robot.camera.applyWhiteBalance(bitmap, whitestPixel);
                    //Save again


                    state = 2;
                    stepStart = this.time;
                }
            }

            else if (state == 2) {
                telemetry.addData("Color", determineJewelColor(bitmap));
                state++;

//                OutputStream output = new FileOutputStream("Phone/DCIM/Camera/Bitmap.bmp");
//                bitmap.compress(Bitmap.CompressFormat.PNG, 90, output);

            }
            else if (state == 3){
                telemetry.addData("Done!", null);
            }

            if (state < 0) {
                //Error occurred if in state -1
                robot.waitForTick(40);
                if (state == -1) {
                    telemetry.update();
                    state = -2;
                }
            }
            else{
                telemetry.addData("Current State:", state);
                telemetry.addData("Time in state:", this.time - stepStart);
                telemetry.addData("Repetitions: x", LoopRep);
                telemetry.update();
                // need to deactive before timeout
            }
        }
        stop_auto();
    }
}
