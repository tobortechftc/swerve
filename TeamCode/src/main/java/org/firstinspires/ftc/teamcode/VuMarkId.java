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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.concurrent.TimeUnit;

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

@Autonomous(name="Vu-Test", group ="Test")
public class VuMarkId extends SwerveUtilLOP {

    //Constants:
    double IMAGE_WIDTH_CROP = 0.33;
    double IMAGE_HEIGHT_CROP = 0.25;
    double IMAGE_OFFSET_X = 0;
    double IMAGE_OFFSET_Y = 0.75;


    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();

        robot.use_swerve = false;
        robot.use_imu = false;
        robot.use_color_sensor = false;
        robot.use_range_sensor = false;
        robot.use_arm = false;
        robot.use_glyph_grabber = false;
        robot.use_relic_grabber = false;
        robot.use_Vuforia = true;

        robot.init(hardwareMap);

        waitForStart();

        if (robot.use_Vuforia) {
            robot.relicTrackables.activate();
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
            if (state == 0) {
                int capacity = robot.vuforia.getFrameQueueCapacity();
                telemetry.addData("6. Vuforia Queue State = ", capacity);
                if (this.time - stepStart > 7) {
                    state = 1;
                    stepStart = this.time;
                    robot.vuforia.setFrameQueueCapacity(1);
                    Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

                }
            } else if (state == 1) {
                int capacity = robot.vuforia.getFrameQueueCapacity();
                telemetry.addData("6. Vuforia Queue State = ", capacity);
                VuforiaLocalizer.CloseableFrame frame = robot.vuforia.getFrameQueue().poll(5, TimeUnit.SECONDS);

                if (frame == null) {
                    state = -1;
                    telemetry.addData("7. ERROR failed to retrieve frame", null);
                    continue;
                }
                Bitmap bitmapTemp = convertFrameToBitmap(frame);
                if (bitmapTemp == null) {
                    state = -1;
                    telemetry.addData("ERROR Failed to retrieve bitmap", null);
                    frame.close();
                    continue;
                }
                state = 2;
                stepStart = this.time;
                bitmap = cropBitmap(bitmapTemp, IMAGE_OFFSET_X, IMAGE_OFFSET_Y, IMAGE_WIDTH_CROP, IMAGE_HEIGHT_CROP);
                frame.close();

//                if (runtime.seconds() > 10 && state == -1) {
//                    if (robot.use_Vuforia) {
//                        robot.use_Vuforia = false;
//                        robot.relicTrackables.deactivate();
//                    }
//                }
            }

            else if (state == 2) {
                telemetry.addData("Bitmap Got", null);
                int height = bitmap.getHeight();
                int width = bitmap.getWidth();
                telemetry.addData("Height:", height);
                telemetry.addData("Width:", width);

                int[] pixels = new int[bitmap.getHeight() * bitmap.getWidth()];

                bitmap.getPixels(pixels, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
                int redTotal = 0;
                int blueTotal = 0;
                int redValue = 0;
                int blueValue = 0;

                for (int pixelI = 0; pixelI < pixels.length; pixelI++) {
                    int b = Color.blue(pixels[pixelI]);
                    int r = Color.red(pixels[pixelI]);

                    if (r > b) {
                        redTotal++;
                        redValue += r;
                    }
                    else {
                        blueTotal++;
                        blueValue += b;
                    }
                }
                if (redTotal > 1.3 * blueTotal) {
                    telemetry.addData("Red Jewel", redTotal);

                }
                else if (blueTotal > 1.3 * redTotal){
                    telemetry.addData("Blue Jewel", blueTotal);

                }
                else {
                    telemetry.addData("ERROR! Threshold not great enough!", null);
                }
                telemetry.addData("Red Total", redTotal);
                telemetry.addData("Blue Total", blueTotal);
                telemetry.addData("Red Value AVG", redTotal > 0 ? redValue/redTotal : 0);
                telemetry.addData("Blue Value AVG", blueTotal > 0 ? blueValue/blueTotal : 0);
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
    Bitmap convertFrameToBitmap(VuforiaLocalizer.CloseableFrame frame) {
        long numImages = frame.getNumImages();
        Image image = null;

        for (int imageI = 0; imageI < numImages; imageI++) {
            if (frame.getImage(imageI).getFormat() == PIXEL_FORMAT.RGB565) {
                image = frame.getImage(imageI);
                break;
            }
        }
        if (image == null) {
            for (int imageI = 0; imageI < numImages; imageI++) {
                telemetry.addData("Image Formats", frame.getImage(imageI).getFormat());
                //For diagnostic purposes
            }
            telemetry.addData("ERROR: failed to find Pixel Format", null);
            return null;

        }
        Bitmap bitmap = Bitmap.createBitmap(image.getWidth(),image.getHeight(), Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(image.getPixels());
        return bitmap;
    }
    Bitmap cropBitmap(Bitmap source, double offset_xF, double offset_yF, double widthF, double heightF) {
        int offset_x = (int)(source.getWidth() * offset_xF);
        int offset_y = (int)(source.getHeight() * offset_yF);
        int width = (int)(source.getWidth() * widthF);
        int height = (int)(source.getHeight() * heightF);
        Bitmap destBitmap = Bitmap.createBitmap(source, offset_x, offset_y, width, height);
        return destBitmap;
    }
}
