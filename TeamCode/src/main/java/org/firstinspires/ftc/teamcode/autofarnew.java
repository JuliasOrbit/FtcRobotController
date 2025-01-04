package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "autofarnew")
//@Disabled
public class autofarnew extends LinearOpMode {

    Hardware2025 robot = new Hardware2025(this);
    private final ElapsedTime runtime = new ElapsedTime();



    public void runOpMode() {
        robot.init();
        waitForStart();
        runtime.reset();
        robot.driveByOtos(5,5,0);
    }
}
