package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Hardware2025 {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor slide = null;
    private DcMotor arm = null;

    // Define IMU object and headings (Make it private so it can't be accessed externally)
    private IMU imu = null;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;

    // Run time (public)
    private final ElapsedTime runtime = new ElapsedTime();

    public Hardware2025(HardwareMap hardwareMap, boolean b) {
    }

    // Color sensing
    public enum sampleColor {RED, YELLOW, BLUE, NONE} //color sensing enum

    private NormalizedColorSensor colorSensor;
    private float colorSensorGain = 20;

    // Magnetic sensing
    public TouchSensor magneticSensorWall;

    public enum SlidePosition {START, WALL, LOW, HIGH, NONE};

    // Slide positions
    public final double WALL_POSITION = 0;
    public final double LOW_POSITION = 5.9;
    public final double HIGH_POSITION = 19.4;
    public SlidePosition slideTargetPosition = SlidePosition.NONE;

    // Magnetic slide sensing for future
    public TouchSensor magneticSensorLow;  // Touch sensor Object
    public TouchSensor magneticSensorHigh;  // Touch sensor Object
    public TouchSensor magneticSensorStart;  // Touch sensor Object

    public void setSlideTargetPosition(SlidePosition slideTargetPosition) {
        this.slideTargetPosition = slideTargetPosition;
    }

    //Drive constants
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: our Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 100.0 / 25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double COUNTS_PER_REVOLUTION_SLIDE = 288;
    static final double SLIDE_GEAR_REDUCTION = 2;
    static final double COUNTS_PER_INCH_SLIDE = (COUNTS_PER_REVOLUTION_SLIDE) /
            (1.3125 * Math.PI * SLIDE_GEAR_REDUCTION);

    private double turnSpeed = 0;
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 5.0;
    static final double OPEN_SERVO_CLAW = 0.7;
    static final double CLOSE_SERVO_CLAW = 0.2;
    private static final double BEAK_OPEN = .6;
    private static final double BEAK_CLOSE = .8;
    private int slideTarget;
    private int armTarget;
    private double slideTimeout;
    private double armTimeout;

    // Claw and beak servos and sensors
    Servo clawServo;
    Servo beakServo;
    TouchSensor touchSensor;  // Touch sensor Object

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware2025(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init(HardwareMap hardwareMap) {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        slide = myOpMode.hardwareMap.get(DcMotor.class, "slide");
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        // Define and Initialize sensors
        colorSensor = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        magneticSensorWall = myOpMode.hardwareMap.get(TouchSensor.class, "magnetic_sensor_wall");
        touchSensor = myOpMode.hardwareMap.get(TouchSensor.class, "sensor_touch");
        clawServo = myOpMode.hardwareMap.get(Servo.class, "claw_servo");
        beakServo = myOpMode.hardwareMap.get(Servo.class, "beak_servo");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        resetHeading();
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

//      magneticSensorLow = myOpMode.hardwareMap.get(TouchSensor.class, "magnetic_sensor_low");
//      magneticSensorHigh = myOpMode.hardwareMap.get(TouchSensor.class, "magnetic_sensor_high");
//      magneticSensorStart = myOpMode.hardwareMap.get(TouchSensor.class, "magnetic_sensor_start");
    }

    // end method initTfod()

    //Goes straight by encoder (takes distance)
    public void straightByEncoder(double speed, double distance, double timeout) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if (myOpMode.opModeIsActive()) {
            DcMotor.RunMode oldMotorMode = leftFrontDrive.getMode();

            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget = rightFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display the data for the driver.
                myOpMode.telemetry.addData("Running to", " lf:%7d lb:%7d rf:%7d rb:%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at", " at lf:%7d lb:%7d rf:%7d rb:%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            stop();
            setMotorMode(oldMotorMode);
            myOpMode.sleep(500);
        }
    }

    public void strafeByEncoder(double speed, double distance, double timeout) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        if (myOpMode.opModeIsActive()) {
            DcMotor.RunMode oldMotorMode = leftFrontDrive.getMode();

            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            // For strafing, left side motors move in opposite direction to right side motors
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH); // Reversed
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH); // Reversed
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display the data for the driver.
                myOpMode.telemetry.addData("Running to", " lf:%7d lb:%7d rf:%7d rb:%7d", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at", " at lf:%7d lb:%7d rf:%7d rb:%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }

            stop();
            setMotorMode(oldMotorMode);
            myOpMode.sleep(500);
        }
    }

    public void driveDiagonalByEncoder(double speed, double distance, double timeout) {
        // Break down the diagonal movement into straight and strafe components
        double straightDistance = distance * Math.cos(Math.PI / 4);  // Diagonal is 45 degrees, so cos(45) = 1/sqrt(2)
        double strafeDistance = distance * Math.sin(Math.PI / 4);    // Similarly, sin(45) = 1/sqrt(2)

        // First, move forward/straight by the appropriate amount
        straightByEncoder(speed, straightDistance, timeout);

        // Then, strafe by the appropriate amount
        strafeByEncoder(speed, strafeDistance, timeout);
    }


    //Drives for a set amount of time (takes time)
    public void driveTimed(double axial, double lateral, double yaw, double time) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRobot(axial, lateral, yaw);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            myOpMode.telemetry.update();
        }
        stop();
    }

    //strafe
    public void strafe(double strafe_power) {
        driveRobot(0.0, strafe_power, 0.0);
    }

    //strafe for a set amount of time (takes time)
    public void strafeTimed(double lateral, double time) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRobot(0, lateral, 0);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            myOpMode.telemetry.update();
        }
        stop();
    }

    //drives robot
    public void driveRobot(double axial, double lateral, double yaw) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        myOpMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        myOpMode.telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }

    //field centric
    public void driveRobotFC(double axial, double lateral, double yaw) {
        double y = axial;
        double x = lateral;
        double rx = yaw;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //radians vs degrees
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;  // Counteract imperfect strafing
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        leftFrontDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    public void setMotorMode(DcMotor.RunMode motorMode) {
        leftFrontDrive.setMode(motorMode);
        leftBackDrive.setMode(motorMode);
        rightFrontDrive.setMode(motorMode);
        rightBackDrive.setMode(motorMode);
    }

    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry
        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;
        // Determine the heading current error
        headingError = targetHeading - robotHeading;
        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            myOpMode.telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, (getRawHeading() - headingOffset)); // Clip the speed to the maximum permitted value.
            myOpMode.telemetry.update();
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            driveRobot(0, 0, turnSpeed);
        }
        stop();
    }

    public void stop() {
        driveRobot(0, 0, 0);
        //    slide(0);
    }

    public void straight(double power) {
        driveRobot(power, 0, 0);
    }

    public void straightTimed(double power, double time) {
        driveTimed(power, 0, 0, time);
    }

    public void startSlide() {
        moveSlide(0.3);
        if (magneticSensorWall.isPressed()) {
            slide.setPower(0.0);
            moveSlide(0.0);
        }
    }

    // Claw stuff
    public void openClaw() {
        clawServo.setPosition(OPEN_SERVO_CLAW);
    }

    public void closeClaw() {
        clawServo.setPosition(CLOSE_SERVO_CLAW);
    }

    // Beak stuff
    public void openBeak() {
        beakServo.setPosition(BEAK_OPEN);
    }

    public void closeBeak() {
        beakServo.setPosition(BEAK_CLOSE);
    }

    public void moveArm(double power) {
        arm.setPower(power);
    }


    public void moveSlide(double power) {
        slide.setPower(power);
    }

    public void moveSlideTimed(double power, double time) {
        slide.setPower(power);
        runtime.reset();
        while (myOpMode.opModeIsActive() && (runtime.seconds() < time)) {
            myOpMode.telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            myOpMode.telemetry.update();
        }
        slide.setPower(0.0);
    }

    public void driveDiagonalForTime(double forwardPower, double strafePower, double time) {
        driveTimed(forwardPower, strafePower, 0, time);
    }

    public sampleColor getColor() {
        colorSensor.setGain(colorSensorGain);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[0] > 1 && hsvValues[0] < 75) {
            myOpMode.telemetry.addData("Red", "%.3f", hsvValues[0]);
            return sampleColor.RED;
        } else if (hsvValues[0] > 75 && hsvValues[0] < 130) {
            myOpMode.telemetry.addData("Yellow", "%.3f", hsvValues[0]);
            return sampleColor.YELLOW;
        } else if (hsvValues[0] > 150 && hsvValues[0] < 280) {
            myOpMode.telemetry.addData("Blue", "%.3f", hsvValues[0]);
            return sampleColor.BLUE;

        }
        myOpMode.telemetry.addData("no color found", 0);
        return sampleColor.NONE;
    }

    public void relativeSlideByEncoder(double speed, double distance, double timeout) {
        // Determine new target position, and pass to motor controller
        slideTimeout = timeout;
        slideTarget = slide.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH_SLIDE);
        slide.setTargetPosition(slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        slide.setPower(Math.abs(speed));
    }

    public void startSlideByEncoder(double speed, double position, double timeout) {
        // Determine new target position, and pass to motor controller
        slideTimeout = timeout;
        slideTarget = (int) (position * COUNTS_PER_INCH_SLIDE);
        slide.setTargetPosition(slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        slide.setPower(Math.abs(speed));
    }

    public boolean isSlideDone() {

        if ((runtime.seconds() < slideTimeout) && (slide.isBusy())) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", " st:%7d ", slideTarget);
            myOpMode.telemetry.addData("Currently at", " at st:%7d", slide.getCurrentPosition());
            myOpMode.telemetry.update();
            return false;

        } else {
            stopSlideEncoder();
            return true;
        }
    }

    public void stopSlideEncoder() {
        slide.setPower(0.01);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isSlideBusy() {
        return slide.isBusy();
    }

    public SlidePosition getSlideCurrent() {
        if (magneticSensorWall.isPressed()) {
            myOpMode.telemetry.addData("LinearSlide", "Is at wall");
            return SlidePosition.WALL;
        } else myOpMode.telemetry.addData("linearSlide", "Is not at wall");

        return null;
    }
    public double getSlidePower() {
        return slide.getPower();
    }

    public void startArmByEncoder(double speed, double position, double timeout) {
        // Determine new target position, and pass to motor controller
        armTimeout = timeout;
        armTarget = (int) (position * COUNTS_PER_INCH_SLIDE);
        arm.setTargetPosition(armTarget);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        arm.setPower(Math.abs(speed));

/* Code for next qualifier
       if (magneticSensorLow.isPressed()) {
            myOpMode.telemetry.addData("LinearSlide", "Is at low");
            return SlidePosition.LOW;
        }

        if (magneticSensorHigh.isPressed()) {
            myOpMode.telemetry.addData("LinearSlide", "Is at high");
            return SlidePosition.HIGH;
        }

           if (magneticSensorStart.isPressed()) {
            myOpMode.telemetry.addData("LinearSlide", "Is at start");
            return SlidePosition.START;
        }

   public void runSlide() {

       if (slideTargetPosition == SlidePosition.START) {

           double power = 0.0;

           switch (getSlideCurrent()) {
               case HIGH:
               case LOW:
               case WALL:

                   power = -0.5;
                   break;

               case NONE:
                   break;
           }
           moveSlide(power);

           if (getSlidePower() > 0.0) {
               switch (slideTargetPosition) {
                   case START:
                       if (magneticSensorStart.isPressed()) {
                           moveSlide(0.0);
                           slideCurrentPosition = Hardware2025.SlidePosition.START;
                       }}
               if (getSlidePower() < 0.0) {

                   switch (slideTargetPosition) {
                       case START:
                           if (magneticSensorStart.isPressed()) {
                               moveSlide(0.0);
                               slideCurrentPosition = Hardware2025.SlidePosition.START; } } } }}

        if (slideTargetPosition == SlidePosition.WALL) {
           //go to Wall position
           double power = 0.0;
           slideTargetPosition = Hardware2025.SlidePosition.WALL;
           switch (getSlideCurrent()) {
               case HIGH:
               case LOW:
                   power = -0.5;
                   break;

               case START:
                   power = 0.5;
                   break;
               case NONE:
                   break;
           }
           moveSlide(power);

           if (getSlidePower() > 0.0) {
               switch (slideTargetPosition) {
                   case WALL:
                       if (magneticSensorWall.isPressed()) {
                           moveSlide(0.0); }}
               if (getSlidePower() < 0.0) {
                   switch (slideTargetPosition) {
                       case WALL:
                           if (magneticSensorWall.isPressed()) {
                               moveSlide(0.0); }}}}}
       if (slideTargetPosition == SlidePosition.LOW) {
           //go to Low Bar position
           double power = 0.0;
           slideTargetPosition = Hardware2025.SlidePosition.LOW;
           switch (getSlideCurrent()) {
               case HIGH:
                   power = -0.5;
                   break;

               case START:
               case WALL:
                   power = 0.5;
                   break;
               case NONE:
                   break;
           }
           moveSlide(power);

           if (getSlidePower() > 0.0) {
               switch (slideTargetPosition) {
                   case LOW:
                       if (magneticSensorLow.isPressed()) {
                           moveSlide(0.0);
                       }}
               if (getSlidePower() < 0.0) {
                   switch (slideTargetPosition) {
                       case LOW:
                           if (magneticSensorLow.isPressed()) {
                               moveSlide(0.0);
                           }} }}}

       if (slideTargetPosition == SlidePosition.HIGH) {
           //go to Low Bar position
           double power = 0.0;
           slideTargetPosition = Hardware2025.SlidePosition.HIGH;
           switch (getSlideCurrent()) {
               case LOW:
               case START:
               case WALL:
                   power = 0.5;
                   break;
               case NONE:
                   break;}
           moveSlide(power);
           if (getSlidePower() > 0.0) {
               switch (slideTargetPosition) {
                   case HIGH:
                       if (magneticSensorHigh.isPressed()) {
                           moveSlide(0.0);
                       }}
               if (getSlidePower() < 0.0) {
                   switch (slideTargetPosition) {
                       case HIGH:
                           if (magneticSensorHigh.isPressed()) {
                               moveSlide(0.0); } } } } }}
*/
}
}






