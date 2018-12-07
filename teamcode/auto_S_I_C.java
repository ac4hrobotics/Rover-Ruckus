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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pigbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class auto_S_I_C extends LinearOpMode
{

    /* Declare OpMode members. */
    HardwarePIGbot         robot   = new HardwarePIGbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     PULLEY_DIAMETER_INCHES   = 1.0 ;     // For figuring circumference of mast and lift

    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415 ); //Math.PI works as well
    //static final double     COUNTS_PER_INCH_MAST    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
   //                                                   (PULLEY_DIAMETER_INCHES * 3.1415 ); //Math.PI works as well
    //static final double     COUNTS_PER_INCH_LIFT    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //                                                  (PULLEY_DIAMETER_INCHES * 3.1415 ); //Math.PI works as well
    static final double     DRIVE_SPEED             = .5;
    static final double     TURN_SPEED              = .3;
    static final double     LIFT_SPEED              = 1;
    static final double     MAST_SPEED              = 1;


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MastMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MastMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                robot.LeftMotor.getCurrentPosition(),
                robot.RightMotor.getCurrentPosition(),
                robot.LiftMotor.getCurrentPosition(),
                robot.MastMotor.getCurrentPosition());





telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // mastInches positive goes up,  Negative goes down
        // liftInches positive goes up,  Negative goes down
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(MAST_SPEED,  0,  0, 0, 20,2 );  // S1: Lower mast 1 Inches with 5 Sec timeout
        encoderDrive(LIFT_SPEED,   0,  0, 0, -50,10);    // S2: Raise lift 5 Inches with 10 Sec timeout
        encoderDrive(DRIVE_SPEED,  48,  48, 0, 0,8);    // S3: Forward 2 Inches with 4 Sec timeout
        robot.Latch.setPosition(.45);            // S4: Stop and close the claw.
        encoderDrive(TURN_SPEED,   24,  -24, 0, 0,8);    // S5: Turn 24 Inches with 4 Sec timeout
      //  encoderDrive(MAST_SPEED,   0,  0, 0, 0,4);    // S6: Reverse 24 Inches with 4 Sec timeout

        
        // robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double liftInches, double mastInches, double timeoutS) {




        int newLeftTarget;
        int newRightTarget;
        int newLiftTarget;
        int newMastTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position for drive, and pass to motor controller
            newLeftTarget  =  robot.LeftMotor.getCurrentPosition()   + (int)(leftInches  * COUNTS_PER_INCH);
            newRightTarget =  robot.RightMotor.getCurrentPosition()  + (int)(rightInches * COUNTS_PER_INCH);
            newLiftTarget  =  robot.LiftMotor.getCurrentPosition()   + (int)(liftInches  * COUNTS_PER_INCH);      // Determine new target position for Lift and pass to motor controller
            newMastTarget  =  robot.MastMotor.getCurrentPosition()   + (int)(mastInches  * COUNTS_PER_INCH);      // Determine new target position for Mast, and pass to motor controller
            robot.LeftMotor.setTargetPosition(newLeftTarget);
            robot.RightMotor.setTargetPosition(newRightTarget);
            robot.LiftMotor.setTargetPosition(newLiftTarget);
            robot.MastMotor.setTargetPosition(newMastTarget);




            // Turn On RUN_TO_POSITION
            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.MastMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();
            robot.LeftMotor.setPower(Math.abs(speed));
            robot.RightMotor.setPower(Math.abs(speed));
            robot.LiftMotor.setPower(Math.abs(speed));
            robot.MastMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && //AND
                    (runtime.seconds() < timeoutS) && //AND
                    (robot.LeftMotor.isBusy() || //OR
                            (robot.RightMotor.isBusy() || //OR
                                    (robot.LiftMotor.isBusy() ||  //OR
                                            (robot.MastMotor.isBusy()))))) {






                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d" , newLeftTarget,  newRightTarget, newLiftTarget, newMastTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",

                        robot.LeftMotor.getCurrentPosition(),
                        robot.RightMotor.getCurrentPosition(),
                        robot.LeftMotor.getCurrentPosition(),
                        robot.MastMotor.getCurrentPosition());
                                           // robot.Arm.getPosition();
                                           // robot.Latch.getPosition();
                                            //robot.Wrist.getPosition();
                                           // robot.Gripper.getPosition();
                                           // robot.Pig.getPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.LeftMotor.setPower(0);
            robot.RightMotor.setPower(0);
            robot.LiftMotor.setPower(0);
            robot.MastMotor.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.MastMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
