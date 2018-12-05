package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *  Motors:
 *  Front and Back Left/Right
 *  Ladder lift ascend/descend
 *  String coil motor.
 */
public class Hardware97654
{
    /* Public OpMode members. */
    public DcMotor motorFrontRight  = null;
    public DcMotor motorFrontLeft   = null;
    public DcMotor motorBackRight   = null;
    public DcMotor motorBackLeft    = null;
    public DcMotor motorLadderlift  = null;
    public DcMotor motorCoil        = null;
    public DcMotor sweeper          = null;
    public DcMotor pa               = null;
    /* public Servo   leftButton = null;
     public Servo   rightButton = null;
 */
    static int EncoderCPR = 1440;
    static double GearRatio = 1;
    static double WheelDiameter = 9.4;
    static int distance = 53;
    /* public static final double RIGHT_INIT_SERVO =  0;
     public static final double LEFT_INIT_SERVO =  1;
     */
    final static double circumference = Math.PI * WheelDiameter;
    final static double rotations = distance / circumference;
    final static double counts = EncoderCPR * rotations * GearRatio;


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware97654() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFrontRight     = hwMap.dcMotor.get("Front Right");
        motorFrontLeft      = hwMap.dcMotor.get("Front Left");
        motorBackLeft       = hwMap.dcMotor.get("Back Left");
        motorBackRight      = hwMap.dcMotor.get("Back Right");
        motorLadderlift     = hwMap.dcMotor.get("Ladder lift");
        motorCoil           = hwMap.dcMotor.get("Coil motor");
        sweeper             = hwMap.dcMotor.get("Sweeper");
        pa                  = hwMap.dcMotor.get("pa");

       /* leftButton          = hwMap.servo.get("bt1");
        rightButton         = hwMap.servo.get("bt2");
*/
        //These work without reversing (Tetrix motors).
        //AndyMark motors may be opposite, in which case uncomment these lines:
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorLadderlift.setPower(0);
        motorCoil.setPower(0);
        sweeper.setPower(0);
        pa.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLadderlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCoil.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       /*
       // Define and initialize ALL installed servos.
        leftButton = hwMap.servo.get("bt1");
        rightButton = hwMap.servo.get("bt2");
        leftButton.setPosition(LEFT_INIT_SERVO);
        rightButton.setPosition(RIGHT_INIT_SERVO);
        */
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
