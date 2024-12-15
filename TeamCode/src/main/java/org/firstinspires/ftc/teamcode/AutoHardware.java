package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoHardware extends Hardware2025 {
    private final LinearOpMode myOpMode;
    public AutoHardware(LinearOpMode opmode) {
        super(opmode.hardwareMap, true);
        this.myOpMode = opmode;

        // Initialize hardware from Hardware2025 using the opmode's hardwareMap
        init(opmode.hardwareMap);
    }

    // Example method to test functionality
    public void driveForward(double speed, int distance) {
        straightByEncoder(speed, distance, 10); // Example: Move forward using encoders
        myOpMode.telemetry.addData("Action", "Driving Forward");
        myOpMode.telemetry.update();
    }


    public void driveWhileTurn(double speed,double angle, double timeout) {

    }


}
