/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains Teleop
 */

@TeleOp(name = "teleop2025", group = "Linear Opmode")

public class teleop2025 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    Hardware2025 robot = new Hardware2025(this);
    private final ElapsedTime runtime = new ElapsedTime();

    // Robot class
    public void runOpMode() {
        robot.init();

        //robot.startSlide();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.driveWithOtos();

            // This button choice was made so that it is hard to hit on accident
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                robot.resetYaw();
            }

            //For telemetry
            robot.getColor();
            robot.getSlideCurrent();
            telemetry.update();

            //Slow scales input
            double gp1LY = gamepad1.left_stick_y;
            double gp1LX = gamepad1.left_stick_x;
            double gp1RX = gamepad1.right_stick_x;

            //slow scale 1
            if (gamepad1.right_bumper) {
                double slowScale = .25;
                gp1LY *= slowScale;
                gp1LX *= slowScale;
                gp1RX *= slowScale;
            }

            //slow scale 2
            if (gamepad1.left_bumper) {
                double slowScale = .33;
                gp1LY *= slowScale;
                gp1LX *= slowScale;
                gp1RX *= slowScale;
            }

            // Field centric
            robot.driveRobotFC(-gp1LY, gp1LX, gp1RX);

            //ignore input or cancel (another method, kill slide/stop slide) currently running operation if second button pressed
            //go to the pickup height

            // Clip on to bar
            if (gamepad2.a) {
                robot.relativeSlideByEncoder(1, -4.5, 10);
            }

            // Go to wall position
            if (gamepad2.b) {
                robot.startSlideByEncoder(.5, robot.WALL_POSITION, 10);
            }

            // Go to low bar height
            if (gamepad2.x) {
                robot.startSlideByEncoder(.5, robot.LOW_POSITION, 10);
            }

            // Go to high bar height
            if (gamepad2.y) {
                robot.startSlideByEncoder(.5, robot.HIGH_POSITION, 10);
            }
            // Checks if the slide is where it should be
            robot.isSlideDone();

            //open and close claw via touch sensor
            if (gamepad2.right_bumper) {
                robot.closeClaw();
            } else {
                robot.openClaw();
            }

            if (gamepad2.left_bumper) {
                robot.closeBeak();
            } else {
                robot.openBeak();
            }

            // Move the arm to pick up a sample
            if (gamepad2.dpad_left) {
                robot.moveArm(1);
                //robot.startArmByEncoder(.5, 5, 10);
            }

            // Move the arm back to the robot
            if (gamepad2.dpad_right) {
                robot.moveArm(-1);
                //robot.startArmByEncoder(.5, -5, 10);
            }

            // Shut off arm power
            else {
                robot.moveArm(0);
            }
            telemetry.update();

        }
    }
}