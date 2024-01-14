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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Blue_outo", group="Robot")
//@Disabled
public class Blue_outo extends LinearOpMode {

    //private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */

    //private static final String TFOD_MODEL_ASSET = "Red_Block_tflite";


    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";

    //private static final String[] LABELS = {
    //        "Red Block",
    //};

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */

    //private VisionPortal visionPortal;

    /* Declare OpMode members. */
    public DcMotor Motor1 = null;
    public DcMotor Motor3 = null;
    public DcMotor Motor2 = null;
    public DcMotor Motor4 = null;

    public Servo Claw1 = null;

    public Servo Claw2 = null;

    public DcMotor LinearSlide = null;

    public Servo Wrist = null;
    public int slide_encoder = 0;

    //public int Motor_encoder_start = 0;

    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.4;
    static final double TURN_SPEED = 0.2;

    @Override
    public void runOpMode() {

        //initTfod();


        // Initialize the drive system variables.
        Motor1 = hardwareMap.get(DcMotor.class, "Motor1");
        Motor3 = hardwareMap.get(DcMotor.class, "Motor3");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");
        LinearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        Claw1 = hardwareMap.get(Servo.class, "Claw1");
        Claw2 = hardwareMap.get(Servo.class, "Claw2");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        Motor1.setDirection(DcMotor.Direction.REVERSE);
        Motor3.setDirection(DcMotor.Direction.REVERSE);
        Motor2.setDirection(DcMotor.Direction.REVERSE);
        Motor4.setDirection(DcMotor.Direction.FORWARD);


        //Motor_encoder_start= Motor1.getCurrentPosition();

        slide_encoder = LinearSlide.getCurrentPosition();
        telemetry.addData("LinearSlide Encoder", "%d", slide_encoder);


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        // double x = 0;

        /*if (opModeIsActive()) {
            while (opModeIsActive()) {

                x = telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }
        */

        //telemetry.addData("Image x:", "%.0f", x);

        // Push telemetry to the Driver Station.
        telemetry.update();
        sleep(5000);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        //Motor_encoder_start = Motor1.getCurrentPosition();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way


        Claw1.setPosition(0);
        Claw2.setPosition(1);


        // Step 1:  Drive forward for 3 seconds
       /* Motor1.setPower(FORWARD_SPEED);
        Motor3.setPower(FORWARD_SPEED);
        Motor2.setPower(FORWARD_SPEED);
        Motor4.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.0) && ((Motor1.getCurrentPosition()- Motor_encoder_start)<600)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.addData("Motor1 Position","%d",(Motor1.getCurrentPosition()- Motor_encoder_start) );
            telemetry.update();
        }*/

        runtime.reset();

        Motor1.setPower(.3);
        Motor2.setPower(.3);
        Motor3.setPower(-.3);
        Motor4.setPower(-.3);

        sleep(3000);


            /*if (x > 1 && x < 100) {

                telemetry.addData("Direction", "left");

                Motor1.setPower(.3);
                Motor3.setPower(.3);
                Motor2.setPower(-.3);
                Motor4.setPower(-.3);

                sleep(2000);

                Motor1.setPower(0);
                Motor3.setPower(0);
                Motor2.setPower(0);
                Motor4.setPower(0);

                while(opModeIsActive() && runtime.seconds()<2) {

                    sleep(1000);

                    Motor1.setPower(.3);
                    Motor3.setPower(.3);
                    Motor2.setPower(.3);
                    Motor4.setPower(.3);

                    sleep(1000);

                    Wrist.setPosition(.17);

                    sleep(1000);

                    Wrist.setPosition(.5);

                    sleep(1000);

                    Motor1.setPower(-.3);
                    Motor3.setPower(-.3);
                    Motor2.setPower(.3);
                    Motor4.setPower(.3);
                }
            }
            else if (x > 100 && x < 500) {

                telemetry.addData("Direction", "straight");
                Motor1.setPower(.3);
                Motor3.setPower(.3);
                Motor2.setPower(.3);
                Motor4.setPower(.3);

                sleep(2000);
                Motor1.setPower(0);
                Motor3.setPower(0);
                Motor2.setPower(0);
                Motor4.setPower(0);

                while(opModeIsActive() && runtime.seconds()<2) {

                    sleep(1000);

                    Motor1.setPower(.3);
                    Motor3.setPower(.3);
                    Motor2.setPower(.3);
                    Motor4.setPower(.3);

                    sleep(1000);

                    Wrist.setPosition(.17);

                    sleep(1000);

                    Wrist.setPosition(.5);

                    sleep(1000);

                    Motor1.setPower(-.3);
                    Motor3.setPower(-.3);
                    Motor2.setPower(.3);
                    Motor4.setPower(.3);
                }
            }
            else if ((x > 500) && x < 640) {

                telemetry.addData("Direction", "right");

                Motor1.setPower(-.3);
                Motor3.setPower(.3);
                Motor2.setPower(-.3);
                Motor4.setPower(.3);

                sleep(2000);

                Motor1.setPower(0);
                Motor3.setPower(0);
                Motor2.setPower(0);
                Motor4.setPower(0);

                while(opModeIsActive() && runtime.seconds()<2) {

                    sleep(1000);

                    Motor1.setPower(.3);
                    Motor3.setPower(.3);
                    Motor2.setPower(.3);
                    Motor4.setPower(.3);

                    sleep(1000);

                    Wrist.setPosition(.17);

                    sleep(1000);

                    Wrist.setPosition(.5);

                    sleep(1000);

                    Motor1.setPower(-.3);
                    Motor3.setPower(-.3);
                    Motor2.setPower(.3);
                    Motor4.setPower(.3);
                }
            }

            else {

                Motor1.setPower(.3);
                Motor3.setPower(.3);
                Motor2.setPower(.3);
                Motor4.setPower(.3);

                sleep(2000);

                Motor1.setPower(.0);
                Motor3.setPower(.0);
                Motor2.setPower(.0);
                Motor4.setPower(.0);

                Wrist.setPosition(.17);

                sleep(1000);

                Wrist.setPosition(.5);
            }*/
        telemetry.update();



        /*Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);

        sleep(10000);
        resetRuntime();

        while (runtime.seconds()<2) {
            Motor1.setPower(-.2);
            Motor4.setPower(.2);
            Motor2.setPower(.2);
            Motor3.setPower(-.2);
        }
        sleep(1000);*/

       // Wrist.setPosition(.2);

        // ato code for moving the claw.

        /*LinearSlide.setPower(.4);
        runtime.reset();
        slide_encoder = LinearSlide.getCurrentPosition();
        while (opModeIsActive() && slide_encoder < 100) {
            slide_encoder = LinearSlide.getCurrentPosition();
            telemetry.addData("LinearSlide Encoder", "%d", slide_encoder);
            telemetry.update();
        }*/

        /*LinearSlide.setPower(-.4);
        runtime.reset();
        slide_encoder = LinearSlide.getCurrentPosition();
        while (opModeIsActive() && slide_encoder > 2) {
            slide_encoder = LinearSlide.getCurrentPosition();
            telemetry.addData("LinearSlide Encoder", "%d", slide_encoder);
            telemetry.update();
        }*/


/*
        // Step 2:  Spin right for 1.3 seconds
        leftDrive.setPower(TURN_SPEED);
        rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backward for 1 Second
        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
*/
        // Step 4:  Stop
        Motor1.setPower(0);
        Motor3.setPower(0);
        Motor2.setPower(0);
        Motor4.setPower(0);
        LinearSlide.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

        //  visionPortal.close();
    }
}

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
   /* private void initTfod() {


        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the TensorFlow processor the easy way.
        //tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     *
     * @return
     */
   /* public double telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        double x = 0;
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            return x;
        }   // end for() loop
        return x;
    }
}*/
