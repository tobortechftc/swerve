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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

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
            telemetry.addData("Repetitions: x", LoopRep);
            if (state == 0) {
                int capacity = robot.vuforia.getFrameQueueCapacity();
                telemetry.addData("6. Vuforia Queue State = ", capacity);
                if (this.time - stepStart > 7) {
                    state = 1;
                    stepStart = this.time;
                    robot.vuforia.setFrameQueueCapacity(1);
                }
            } else if (state == 1) {
                int capacity = robot.vuforia.getFrameQueueCapacity();
                telemetry.addData("6. Vuforia Queue State = ", capacity);
                VuforiaLocalizer.CloseableFrame frame = robot.vuforia.getFrameQueue().poll(5, TimeUnit.SECONDS);
                if (frame == null) {
                    state = -1;
                    telemetry.addData("7. ERROR State:", state);
                    continue;
                }
                else {
                    state = 2;
                    stepStart = this.time;
                    continue;
                }
//                if (runtime.seconds() > 10 && state == -1) {
//                    if (robot.use_Vuforia) {
//                        robot.use_Vuforia = false;
//                        robot.relicTrackables.deactivate();
//                    }
//                }
            }
            else if (state == 2) {
                telemetry.addData("Frame Got", null);
            }
            telemetry.addData("Current State:", state);
            telemetry.addData("Time in state:", this.time - stepStart);
            telemetry.update();
            // need to deactive before timeout

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
        Bitmap bitmap = Bitmap.createBitmap(image.getWidth(),image.getHeight(), Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(image.getPixels());
        return bitmap;
    }
}
