package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Encoder Blue Ball-Corner-Sweep", group="Blue")
public class TestAuto2 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware97654 robot   = new Hardware97654();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0;     // For figuring circumference

    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     SWEEPER_SPEED           = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sweeper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d  :%7d  :%7d",
                robot.motorFrontRight.getCurrentPosition(),
                robot.motorBackLeft.getCurrentPosition(),
                robot.motorBackRight.getCurrentPosition(),
                robot.motorFrontLeft.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // sleep(10000);


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // FL, BR, FR, BL; Circumferenceâ‰ˆ65.97
        // Aim with red strip on wood bar.

        //  To move the robot:
        //  Move Forward using two wheels:
        //       -x, x, 0, 0  forward

        encoderDrive    (DRIVE_SPEED,  -55,  55,   0,   0, 0, 0, 5.0);  // S1: Forward X Inches with 5 Sec timeout
        //encoderDrive    (DRIVE_SPEED,    0,   0, -35,  35, 0, 0, 4.0);  // S3: Forward X Inches with 4 Sec timeout
        encoderDrive    (TURN_SPEED,   -19, -19, -19, -19, 0, 0, 4.0);  // S2: Turn Right (negative) X Inches with 4 Sec timeout
        encoderDrive    (DRIVE_SPEED,   -32,  32, 32, -32, 0, 0, 4.0);  // this travels forwards on omni wheels
        // encoderDrive    (TURN_SPEED,   -4, -4, -4, -4, 0, 0, 4.0);      // S2: Turn Right (negative) X Inches with 4 Sec timeout
        encoderDrive    (DRIVE_SPEED,   -15,  15, 15, -15, 0, 0, 4.0);  // this travels forwards on omni wheels
        encoderDrive   (SWEEPER_SPEED,  0,   0,   0,   0, -20, 0, 5.0); // this travels forwards on omni wheels
        sleep(500);
        encoderDrive(SWEEPER_SPEED,   0,  0,  0,  0, -20, -1, 5.0);  // this sweeps the balls out

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
    public void encoderDrive(double speed,
                             double FleftInches, double BrightInches,double FrightInches, double BleftInches, double swpr, double pa,
                             double timeoutS) throws InterruptedException {
        int newFLeftTarget;
        int newFRightTarget;
        int newBRightTarget;
        int newBLeftTarget;
        int swpr2Target;
        int paTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLeftTarget  = robot.motorFrontLeft.getCurrentPosition() + (int)(FleftInches * COUNTS_PER_INCH);
            newBRightTarget = robot.motorBackRight.getCurrentPosition() + (int)(BrightInches * COUNTS_PER_INCH);
            newFRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (FrightInches * COUNTS_PER_INCH);
            newBLeftTarget  = robot.motorBackLeft.getCurrentPosition() + (int) (BleftInches* COUNTS_PER_INCH);

            swpr2Target     =  robot.sweeper.getCurrentPosition() + (int) (swpr * COUNTS_PER_INCH);
            paTarget     =  robot.pa.getCurrentPosition() + (int) (pa * COUNTS_PER_INCH);

            robot.motorBackLeft.setTargetPosition(newBLeftTarget);
            robot.motorFrontRight.setTargetPosition(newFRightTarget);
            robot.motorFrontLeft.setTargetPosition(newFLeftTarget);
            robot.motorBackRight.setTargetPosition(newBRightTarget);

            robot.sweeper.setTargetPosition(swpr2Target);
            robot.pa.setTargetPosition(paTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.sweeper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pa.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFrontLeft.setPower(Math.abs(-speed));
            robot.motorFrontRight.setPower(Math.abs(speed));
            robot.motorBackLeft.setPower(Math.abs(-speed));
            robot.motorBackRight.setPower(Math.abs(speed));
            robot.sweeper.setPower(Math.abs(-speed));
            robot.pa.setPower(Math.abs(-speed));


            // keep looping while we are still active, and there is time left, and (was both) any of the motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((robot.motorFrontRight.isBusy()&& robot.motorBackLeft.isBusy()) || (robot.motorBackRight.isBusy() && robot.motorFrontLeft.isBusy()) || robot.sweeper.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Run to %7d :%7d :%7d :%7d :%7d :%7d", newFRightTarget, newBLeftTarget, newBRightTarget, newFLeftTarget, swpr2Target, paTarget );
                telemetry.addData("Path2",  "Run at %7d :%7d :%7d :%7d :%7d :%7d",
                        robot.motorFrontRight.getCurrentPosition(),
                        robot.motorBackLeft.getCurrentPosition(),
                        robot.motorBackRight.getCurrentPosition(),
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.sweeper.getCurrentPosition(),
                        robot.pa.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);
            robot.motorFrontLeft.setPower(0);
            robot.sweeper.setPower(0);
            robot.pa.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.sweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.pa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
