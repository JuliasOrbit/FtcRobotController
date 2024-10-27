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
//@Disabled
public class tele2025ang extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    hardware2025Ang robot = new hardware2025Ang(this);
    private final ElapsedTime runtime = new ElapsedTime();

    // @Override
    public void runOpMode() {
        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            // This button choice was made so that it is hard to hit on accident
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                robot.resetYaw();
            }

            //SlowScales
            double gp1LY = gamepad1.left_stick_y;
            double gp1LX = gamepad1.left_stick_x;
            double gp1RX = gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                double slowscale = .25;
                gp1LY *= slowscale;
                gp1LX *= slowscale;
                gp1RX *= slowscale;
            }

            if (gamepad1.left_bumper) {
                double slowscale = .1;
                gp1LY *= slowscale;
                gp1LX *= slowscale;
                gp1RX *= slowscale;
            }
            robot.driveRobotFC(-gp1LY, gp1LX, gp1RX);

            if (gamepad2.dpad_up) {
                robot.moveSlide(1);
            }

            if (gamepad2.dpad_down) {
                robot.moveSlide(-1);
            }
        }
    }

    private void driveRobotFC(double v, double gp1LX, double gp1RX) {}
    private void resetYaw() {}
}