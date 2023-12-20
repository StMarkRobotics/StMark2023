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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Example", group="Robot")
//@Disabled
public class Example extends OpMode{

    /* Declare OpMode members. */
    public DcMotor  Motor1   = null;
    public DcMotor  Motor3    =null;
    public DcMotor  Motor2  = null;
    public DcMotor  Motor4  = null;
  //  public DcMotor  leftArm     = null;
    public Servo Launcher    = null;
   // public Servo    rightClaw   = null;
    public DcMotor  LinearSlide =null;
    public Servo Claw1 =null;
    public Servo Claw2 =null;
    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    //public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

    public  int slide_encoder = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        Motor1  = hardwareMap.get(DcMotor.class, "Motor1");
        Motor3  = hardwareMap.get(DcMotor.class, "Motor3");
        Motor2 = hardwareMap.get(DcMotor.class, "Motor2");
        Motor4 = hardwareMap.get(DcMotor.class, "Motor4");
       // leftArm    = hardwareMap.get(DcMotor.class, "left_arm");
        LinearSlide    = hardwareMap.get(DcMotor.class, "LinearSlide");
        Launcher = hardwareMap.get(Servo.class, "Launcher");
        Claw1 = hardwareMap.get(Servo.class, "Claw1");
        Claw2 = hardwareMap.get(Servo.class, "Claw2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Motor1.setDirection(DcMotor.Direction.REVERSE);
        Motor3.setDirection(DcMotor.Direction.REVERSE);
        Motor2.setDirection(DcMotor.Direction.REVERSE);
        Motor4.setDirection(DcMotor.Direction.FORWARD);

        // encoder for linear slide

        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Claw 1 and Claw 2
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        //rightClaw = hardwareMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        LinearSlide.setPower(0);
        slide_encoder=LinearSlide.getCurrentPosition();
        telemetry.addData("LinearSlide Encoder", "%d", slide_encoder);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double y1;
        double x1;
        double x2;
        double y2;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        y1 = -gamepad1.left_stick_y;
        x1 = -gamepad1.left_stick_x;
        x2 = -gamepad1.right_stick_x;
        y2=-gamepad1.right_stick_y;


        /*Motor1.setPower(y1+x1+x2);
        Motor3.setPower(y1-x1+x2);
        Motor2.setPower(y1+x1-x2);
        Motor4.setPower(y1-x1-x2);*/

        Motor1.setPower(y1-x1+x2);
        Motor3.setPower(y1+x1-x2);
        Motor2.setPower(y1+x1+x2);
        Motor4.setPower(y1-x1-x2);

    // airplane launcher
        if (gamepad1.x)
            Launcher.setPosition(.8);
        else
            Launcher.setPosition(.5);

        if(gamepad2.right_bumper){
            //Claw Open
            Claw1.setPosition(0.5);
            Claw2.setPosition(0.5);
        }
        if (gamepad2.left_bumper) {
            //Claw Close
            Claw1.setPosition(0.3);
            Claw2.setPosition(0.3);
        }


        // Use gamepad left & right Bumpers to open and close the claw
       // if (gamepad1.right_bumper)
           // clawOffset += CLAW_SPEED;
       // else if (gamepad1.left_bumper)
           // clawOffset -= CLAW_SPEED;

        // code that controles LinearSlide
        slide_encoder=LinearSlide.getCurrentPosition();
        if (gamepad1.y & slide_encoder<350)
            LinearSlide.setPower(.4);
        else if (gamepad1.a & slide_encoder>20)
            LinearSlide.setPower(-.4);
        else
            LinearSlide.setPower(0);

        // code that controles Launcher
        //if (Launcher.setPosition(0.7);


        // Move both servos to new position.  Assume servos are mirror image of each other.
       // clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        //leftClaw.setPosition(MID_SERVO + clawOffset);
       // rightClaw.setPosition(MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
       // if (gamepad1.y)
       //     leftArm.setPower(ARM_UP_POWER);
      //  else if (gamepad1.a)
       //     leftArm.setPower(ARM_DOWN_POWER);
       // else
           // leftArm.setPower(0.0);

        // Send telemetry message to signify robot running;
        // telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        //telemetry.addData("y1",  "%.2f", y1);
        //telemetry.addData("x1", "%.2f", x1);
        //telemetry.addData("x2", "%.2f", x2);
        telemetry.addData("LinearSlide Encoder", "%d", slide_encoder);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
