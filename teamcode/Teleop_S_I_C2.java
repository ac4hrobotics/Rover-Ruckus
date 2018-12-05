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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop_S_I_C2", group="Pushbot")
//@Disabled
public class Teleop_S_I_C2 extends LinearOpMode {

    private DcMotor RightMotor;
    private DcMotor LeftMotor;
    private DcMotor LiftMotor;
    private Servo Pig;
    private Servo Latch;
    private DcMotor MastMotor;
    private Servo Arm;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double GPY;
        double GPX;

        RightMotor = hardwareMap.dcMotor.get("Right Motor ");
        LeftMotor = hardwareMap.dcMotor.get("Left Motor");
        LiftMotor = hardwareMap.dcMotor.get("Lift Motor ");
        Pig = hardwareMap.servo.get("Pig");
        Latch = hardwareMap.servo.get("Latch ");
        MastMotor = hardwareMap.dcMotor.get("Mast Motor ");
        Arm = hardwareMap.servo.get("Arm");

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        RightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            Initservo();
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                GPY = -gamepad1.left_stick_y / 1.5;
                GPX = -gamepad1.right_stick_y / 1.5;
                Latch2();
                if (gamepad2.dpad_down) {
                    Eject_Pig();
                }
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                LeftMotor.setPower(GPY);
                RightMotor.setPower(GPX);
                LiftMotor.setPower(gamepad2.left_stick_y);
                telemetry.addData("Left Pow", LeftMotor.getPower());
                telemetry.addData("Right Pow", RightMotor.getPower());
                telemetry.addData("GamepadValueY", GPY);
                telemetry.addData("GamepadValueX", GPX);
                telemetry.addData("Servo Pig value", Pig.getPosition());
                telemetry.addData("Lift posit", LeftMotor.getCurrentPosition());
                telemetry.addData("Lift power", LeftMotor.getPower());
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Latch2() {
        if (gamepad2.b) {
            Latch.setPosition(0.45);
        }
        if (gamepad2.y) {
            Latch.setPosition(0.27);
        }
        Pig.scaleRange(0.25, 0.5);
    }

    /**
     * Describe this function...
     */
    private void Eject_Pig() {
        Pig.scaleRange(0.2, 0.99);
        Pig.setPosition(1);
        Pig.setPosition(0.4);
    }

    /**
     * Describe this function...
     */
    private void Initservo() {
        Pig.setPosition(0);
        Latch.setPosition(0.45);
        Arm.setPosition(0.5);
    }
}
