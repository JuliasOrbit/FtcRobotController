package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Hardware2025 {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor slide = null;
  //  private DcMotor leftSlide = null;
  //  private DcMotor rightSlide = null;
    private DcMotor arm = null;

    // Define IMU object and headings (Make it private so it can't be accessed externally)
    public IMU imu = null;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;

    // Run time (public)
    private final ElapsedTime runtime = new ElapsedTime();

    // Color sensing
    public enum sampleColor {RED, YELLOW, BLUE, NONE} //color sensing enum

    private NormalizedColorSensor colorSensor;
    private float colorSensorGain = 20;

    // Magnetic sensing
    public TouchSensor magneticSensorWall;

    public enum SlidePosition {START, WALL, LOW, HIGH, NONE}

    ;

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
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
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

    // Create an instance of the otos sensor
    SparkFunOTOS myOtos;


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
    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        slide = myOpMode.hardwareMap.get(DcMotor.class, "slide");
      //  leftSlide = myOpMode.hardwareMap.get(DcMotor.class, "left_slide");
       // rightSlide = myOpMode.hardwareMap.get(DcMotor.class, "right_slide");
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        myOtos = myOpMode.hardwareMap.get(SparkFunOTOS.class, "sensor_otos"); //Otos sensor
        configureOtos();
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
      //  leftSlide.setDirection(DcMotor.Direction.FORWARD); //needs to be fixed, just as a placeholder to not cause errors
      //  rightSlide.setDirection(DcMotor.Direction.FORWARD); //needs to be fixed, just as a placeholder to not cause errors
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
    }
    // end method initTfod()

    //Configuring the otos sensor- for more detailed comments, refer to the SensorSparkFunOTOS.java file
    private void configureOtos() {
        myOpMode.telemetry.addLine("Configuring OTOS...");
        myOpMode.telemetry.update();

        // Set the desired units for linear and angular measurements- currently set to the default inches and degrees
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        //Specify the offset for the sensor relative to the center of the robot.- chnsge
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(.25, 1.1, 180);
        myOtos.setOffset(offset);

        // Set the linear and angular scalars, to compensate for scaling issues with the sensor measurements.
        myOtos.setLinearScalar(49.0/50.0);
        myOtos.setAngularScalar(720.0/719.0);
        myOtos.calibrateImu();

        // Reset the tracking algorithm (resets position to origin)- can be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS reports the robot is at origin. If you do not start at the origin, set the OTOS location to match.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        myOpMode.telemetry.addLine("OTOS configured! Press start to get position data!");
        myOpMode.telemetry.addLine();
        myOpMode.telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        myOpMode.telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        myOpMode.telemetry.update();
    }

    public void driveXYByOtos(double distanceToTravelX, double distanceToTravelY) {
        //May have messed stuff up with taking out too much or leaving too much of the angle in
        //To drive straight, just put in zero for one of the x or y
        double targetHeading = 0;
        SparkFunOTOS.Pose2D pos = myOtos.getPosition(); // Get position

        Translation2d currentT = new Translation2d(pos.x, pos.y);
        Rotation2d currentR = new Rotation2d(pos.h);
        Pose2d current = new Pose2d(currentT, currentR);

        Translation2d targetT = new Translation2d(distanceToTravelX, distanceToTravelY);
        Rotation2d targetR = new Rotation2d(targetHeading);
        Pose2d target = new Pose2d(targetT, targetR);

        Translation2d toTravelT = target.getTranslation().minus(current.getTranslation());
        Rotation2d toTravelR = target.getRotation().minus(current.getRotation());

        Log.i("driveByOtos", current.toString() + target.toString() + toTravelT.toString()+ toTravelR.toString());

        double thresholdDistance = 1.0; // Distance threshold for stopping (x and y)

        //Variables for loop (as to not define the variables inside the loop)
        double correctionX;
        double correctionY;

        //Loop to see where we are and need to go
        while (myOpMode.opModeIsActive() && (Math.abs(toTravelT.getX()) > thresholdDistance || Math.abs(toTravelT.getY()) > thresholdDistance)) {
            correctionX = 0.1*toTravelT.getX(); //X distance needed to go (to be passed into FC function)
            correctionY = 0.1*toTravelT.getY(); //Y distance needed to go (to be passed into FC function)

            driveRobotFC(correctionY, correctionX, 0); //Pass values to the FC function

            pos = myOtos.getPosition();

            currentT = new Translation2d(pos.x, pos.y);
            currentR = new Rotation2d(pos.h);
            current = new Pose2d(currentT, currentR);

            toTravelT = target.getTranslation().minus(current.getTranslation());

            myOpMode.telemetry.addData("Target Axial/Lateral/Yaw", "%5.2f / %5.2f / %5.2f", pos.x, pos.y, pos.h);
            myOpMode.telemetry.update();
        }
        stop();
    }
    public void driveByOtos(double distanceToTravelX, double distanceToTravelY, double targetHeading) {

        SparkFunOTOS.Pose2D pos = myOtos.getPosition(); // Get position

        Translation2d currentT = new Translation2d(pos.x, pos.y);
        Rotation2d currentR = new Rotation2d(pos.h);
        Pose2d current = new Pose2d(currentT, currentR);

        Translation2d targetT = new Translation2d(distanceToTravelX, distanceToTravelY);
        Rotation2d targetR = new Rotation2d(targetHeading);
        Pose2d target = new Pose2d(targetT, targetR);

        Translation2d toTravelT = target.getTranslation().minus(current.getTranslation());
        Rotation2d toTravelR = target.getRotation().minus(current.getRotation());

        Log.i("driveByOtos", current.toString() + target.toString() + toTravelT.toString()+ toTravelR.toString());

        //Calculate the total distances needed to travel
        //double toTravelH = pos.h + targetHeading;
        //double toTravelX = pos.x + distanceToTravelX;
        //double toTravelY = pos.y + distanceToTravelY;

        //Removed: adjustedAxial adjustedLateral adjustedYaw distancedTraveledParallelCurrent distancedTraveledParallelOld distanceTraveledH
        //inchesTraveledParallel distanceTraveledPerpendicularCurrent distanceTraveledPerpendicularOld inchesTraveledPerpendicular
       // double distanceTraveledX = 0.0; // accumulated distance
        //double distanceTraveledY = 0.0; // accumulated distance

        double thresholdDistance = 1.0; // Distance threshold for stopping (x and y)
        double angleThreshold = 1.0; //Angle threshold for stopping

        //Proportional gain removed due to less error?
        double maxSpeed = .25; // Max robot speed

        //Variables for loop (as to not define the variables inside the loop)
        double correctionH;
        double correctionX;
        double correctionY;
        double max;
        double botHeading;
        double distanceTraveledPerpendicular;
        double distancedTraveledParallel;
        //correctionW = getSteeringCorrection(toTravelR, P_TURN_GAIN); //Angle needed to turn (to be passed into FC function)


        //Loop to see where we are and need to go
        while (myOpMode.opModeIsActive() && (Math.abs(toTravelT.getX()) > thresholdDistance || Math.abs(toTravelT.getY()) > thresholdDistance || Math.abs(toTravelR.getDegrees()) > angleThreshold)) {
            //Todo: may need to fuss with the () in the X and Y for casting and dividing purposes
            correctionX = 0.1*toTravelT.getX(); //X distance needed to go (to be passed into FC function)
            correctionY = 0.1*toTravelT.getY(); //Y distance needed to go (to be passed into FC function)
            correctionH = toTravelR.getDegrees();
            //correctionH = 0.001*toTravelR.getDegrees();

            //todo: look for angles and radians
            //todo: pick the equation thing out of the three

            /** max = Math.max(Math.abs(correctionY), Math.abs(correctionX));
            max = Math.max(Math.abs(max), maxSpeed );
            if (max > maxSpeed) { //ensure that the robot does not go past its max and wheels turn at the same rate
                correctionX /= max;
                correctionY /= max;
            } **/

            driveRobotFC(correctionY, correctionX, correctionH); //Pass values to the FC function
            Log.i("driveWhileTurn", String.format("adjustedAxial = %f, adjustedLateral = %f, correctionW = %f", correctionY, correctionX, correctionH));
            // Now we need to calculate distance traveled based on our current angle
            // This duplicates the trigonometry already done in driveRobotFC
            // todo: combine odometry into driverobotFC to elimiate redundancy

            pos = myOtos.getPosition();
            //creating a new one inside every time- may cause error we shall see
            currentT = new Translation2d(pos.x, pos.y);
            currentR = new Rotation2d(pos.h);
            current = new Pose2d(currentT, currentR);

            toTravelT = target.getTranslation().minus(current.getTranslation());
            toTravelR = target.getRotation().minus(current.getRotation());

            //botHeading = pos.h;
            //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); May need to fuss with radians and degrees
            Log.i("driveWhileTurn", String.format("toTravelR = %f", toTravelR.getDegrees()));

            // Rotate the movement direction counter to the bot's rotation
            //this is not correct anyways- distancedTraveledParallelCurrent = leftFrontDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition() + leftFrontDrive.getCurrentPosition();

//            distanceTraveledX = currentT.getX() - startT.getX();
//            distanceTraveledY = currentT.getY() - startT.getY();
//            Log.i("driveWhileTurn", String.format("distanceTraveledX = %f, distanceTraveledY = %f", distanceTraveledX, distanceTraveledY));

           // distancedTraveledParallel = toTravelX - distanceTraveledX;
            //distanceTraveledPerpendicular = toTravelY - distanceTraveledY;

            //deleted using ticks and converting them into inches

            //converting into new reference frame- three options, first prob works?
            // reversing sign of terms
            //distanceTraveledX += distancedTraveledParallel * Math.cos(-botHeading) + distanceTraveledPerpendicular * Math.sin(-botHeading);
            //distanceTraveledY += distancedTraveledParallel * Math.sin(-botHeading) - distanceTraveledPerpendicular * Math.cos(-botHeading);
            // use positive botHeading

            //added to modify distance to travel x minus or plus
            //distanceToTravelX = distanceToTravelX + distanceTraveledX;
            //distanceToTravelY = distanceToTravelY +  distanceTraveledY;
            Log.i("driveWhileTurn", String.format(" distanceToTravelX= %f, distanceToTravelY = %f", distanceToTravelX, distanceToTravelY));

            //Resseting old ticks to current ticks so next time through the loop its current (dont need?)

            // Update telemetry
            myOpMode.telemetry.addData("Target Axial/Lateral/Yaw", "%5.2f / %5.2f / %5.2f", pos.x, pos.y, pos.h);
            myOpMode.telemetry.addData("toTravelR", "%5.2f", toTravelR.getDegrees());
            myOpMode.telemetry.update();
        }
        stop();
    }


    public void driveWithOtos() {
        SparkFunOTOS.Pose2D pos = myOtos.getPosition(); // Get position

        myOpMode.telemetry.addData("X coordinate", pos.x); // Log the position to the telemetry
        myOpMode.telemetry.addData("Y coordinate", pos.y);
        myOpMode.telemetry.addData("Heading angle", pos.h);

        //myOpMode.telemetry.update();  Update the telemetry on the driver station
    }

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
                //myOpMode.telemetry.update();
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
                //myOpMode.telemetry.update();
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
            //myOpMode.telemetry.update();
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

    //figure out degrees v radians stuff also this assumes strafe efficency of 100% sooo prob need to tweak this
    public void driveWhileTurn(double targetAxial, double targetLateral, double targetYaw) {

        double initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // Starting angle relative to robot
        double angleToTravelW = initialHeading - targetYaw; //initialize angle target (changed to add subtraction)
        double distanceToTravelX = targetLateral; //initialize lateral target
        double distanceToTravelY = targetAxial; //initialize axial target
        double distanceTraveledX = 0.0; // accumulated distance
        double distanceTraveledY = 0.0; // accumulated distance

        double leftFrontTicks = leftFrontDrive.getCurrentPosition(); //Capturing the current motor ticks
        double rightFrontTicks = rightFrontDrive.getCurrentPosition();
        double leftBackTicks = leftBackDrive.getCurrentPosition();
        double rightBackTicks = rightBackDrive.getCurrentPosition();

        double thresholdDistance = 1.0; // Distance threshold for stopping (double will rarely ever reach zero)
        double angleThreshold = 5.0; //Angle threshold for stopping (double will rarely ever reach zero)

        double kX = 0.05; // Proportional gain value for X (may need to be greater)
        double kY = 0.05; // Proportional gain value for Y (may need to be greater)
        double maxSpeed = .25; // Max robot speed

        //Variables for loop (as to not define the variables inside the loop)
        double correctionW;
        double correctionX;
        double correctionY;
        double adjustedAxial;
        double adjustedLateral;
        double adjustedYaw;
        double max;
        double botHeading;
        double distancedTraveledParallelCurrent;
        double distancedTraveledParallelOld;
        double inchesTraveledParallel;
        double distanceTraveledPerpendicularCurrent;
        double distanceTraveledPerpendicularOld;
        double inchesTraveledPerpendicular;

        //Loop to see where we are and need to go
        while (myOpMode.opModeIsActive() && (Math.abs(distanceToTravelX) > thresholdDistance || Math.abs(distanceToTravelY) > thresholdDistance || Math.abs(angleToTravelW) > angleThreshold)) {
            //Todo: may need to fuss with the () in the X and Y for casting and dividing purposes
            correctionW = getSteeringCorrection(angleToTravelW, P_TURN_GAIN); //W needed to turn (pass into FC function)
            correctionX = (distanceToTravelX - distanceTraveledX) * kX; //X distance needed to go (pass into FC function)
            correctionY = (distanceToTravelY - distanceTraveledY) * kY; //Y distance needed to go (pass into FC function)

//todo: look for angles and radians
            //todo: pick the equation thing out of the three

            max = Math.max(Math.abs(correctionY), Math.abs(correctionX));
            if (max > maxSpeed) { //ensure that the robot does not go past its max and wheels are turning at same rate
                correctionY /= max;
                correctionX /= max;
            }

            driveRobotFC(correctionY, correctionX, correctionW); //Pass values to the FC function
            Log.i("driveWhileTurn", String.format("adjustedAxial = %f, correctionX = %f, correctionW = %f", correctionY, correctionX, correctionW));
            // Now we need to calculate distance traveled based on our current angle
            // This duplicates the trigonometry already done in driveRobotFC
            // todo: combine odometry into driverobotFC to elimiate redundancy

            //trying only looking at one motor (left frount)
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // May need to fuss with radians and degrees
            Log.i("driveWhileTurn", String.format("botHeading = %f", botHeading));

            // Rotate the movement direction counter to the bot's rotation
            distancedTraveledParallelCurrent = leftFrontDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition() + leftFrontDrive.getCurrentPosition();

            //distancedTraveledParallelOld = leftFrontTicks + rightFrontTicks + leftBackTicks + rightBackTicks;
            distancedTraveledParallelOld = leftFrontTicks;

            //inchesTraveledParallel = ((distancedTraveledParallelCurrent - distancedTraveledParallelOld) /4.0 /COUNTS_PER_INCH);
            inchesTraveledParallel = ((distancedTraveledParallelCurrent - distancedTraveledParallelOld) / COUNTS_PER_INCH);

            distanceTraveledPerpendicularCurrent = (leftFrontDrive.getCurrentPosition() + rightFrontDrive.getCurrentPosition()) - (rightBackDrive.getCurrentPosition() - leftBackDrive.getCurrentPosition());
            distanceTraveledPerpendicularOld = (leftFrontTicks + rightFrontTicks) - (rightBackTicks - leftBackTicks);
            inchesTraveledPerpendicular = ((distanceTraveledPerpendicularCurrent - distanceTraveledPerpendicularOld) / 4.0 / COUNTS_PER_INCH);
            Log.i("driveWhileTurn", String.format("inchesTraveledPerpendicular = %f, inchesTraveledParallel = %f", inchesTraveledPerpendicular, inchesTraveledParallel));

            //converting into new reference frame- the signs here may need to be adjusted
            distanceTraveledX += inchesTraveledParallel * Math.cos(-botHeading) + inchesTraveledPerpendicular * Math.sin(-botHeading);
            distanceTraveledY += inchesTraveledParallel * Math.sin(-botHeading) - inchesTraveledPerpendicular * Math.cos(-botHeading);

           distanceTraveledX += inchesTraveledParallel * Math.cos(-botHeading) - inchesTraveledPerpendicular * Math.sin(-botHeading);
           distanceTraveledY += inchesTraveledParallel * Math.sin(-botHeading) + inchesTraveledPerpendicular * Math.cos(-botHeading);
            Log.i("driveWhileTurn", String.format("distanceTraveledX = %f, distanceTraveledY = %f", distanceTraveledX, distanceTraveledY));

            //added to modify distance to travel x
            distanceToTravelX = distanceToTravelX - distanceTraveledX;
            distanceToTravelY = distanceToTravelY - distanceTraveledY;
            Log.i("driveWhileTurn", String.format(" distanceToTravelX= %f, distanceToTravelY = %f", distanceToTravelX, distanceToTravelY));

            //Resseting old ticks to current ticks so next time through the loop its current
            leftFrontTicks = leftFrontDrive.getCurrentPosition();
            rightFrontTicks = rightFrontDrive.getCurrentPosition();
            leftBackTicks = leftBackDrive.getCurrentPosition();
            rightBackTicks = rightBackDrive.getCurrentPosition();

            // Update telemetry
            myOpMode.telemetry.addData("Target Axial/Lateral/Yaw", "%5.2f / %5.2f / %5.2f", targetAxial, targetLateral, targetYaw);
            myOpMode.telemetry.addData("Remaining Axial/Lateral/Yaw", "%5.2f / %5.2f / %5.2f", distanceToTravelY - distanceTraveledY, distanceToTravelX - distanceTraveledX, headingError);
            myOpMode.telemetry.update();
        }
        stop();
    }

    public void stop() {
        driveRobot(0, 0, 0);
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
          //  myOpMode.telemetry.update();
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
    }
}






