package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "autofarnew")
//@Disabled
public class autofarnew extends LinearOpMode {

    Otos2025 robot = new Otos2025(this);
    private final ElapsedTime runtime = new ElapsedTime();



    public void runOpMode() {
        robot.init();
        waitForStart();
        runtime.reset();
//        robot.driveByOtos(0.0,5.0,0.0);
//        robot.driveByOtos(0.0,0.0,30.0);
        robot.driveByOtos(10,10,90);
        robot.driveByOtos(-10,-10,0);
        sleep(200);

//        robot.driveByOtos(5.,5.,30.0);
//        robot.driveByOtos(-10.,0.,0);
//        robot.driveByOtos(-5.,-5.,0);
    }
}
