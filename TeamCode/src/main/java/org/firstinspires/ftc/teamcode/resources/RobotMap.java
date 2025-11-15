package org.firstinspires.ftc.teamcode.resources;

import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Class that stores references to the motors, servos, sensors, and more of the robot.<br/>
 * To use, first initialize the map with your hardware map, usually defined as <code>hardwareMap</code>:
 *
 * <pre><code>
 *     RobotMap.init(hardwareMap);
 * </code></pre><br/>
 * <p>
 * Initializing gets the hardware from the given map sets all necessary settings needed for the hardware to work.<br/>
 * Then, you can reference the motors using the class name.
 * For instance,
 * <pre><code>
 *     RobotMap.rightFront
 * </code></pre>
 * gets the right front drive motor.
 */
public class RobotMap {

    // Mecanum wheel motors
    public static DcMotorEx leftFront = null;
    public static DcMotorEx rightFront = null;
    public static DcMotorEx leftBack = null;
    public static DcMotorEx rightBack = null;

    // Intake motor
    public static DcMotorEx intake = null;

    // Belt servo
    public static CRServo belt;

    // Kick servo
    public static CRServo kick;

    // Flywheels
    public static DcMotorEx flywheelLeft;
    public static DcMotorEx flywheelRight;

    // Apriltag detection variables
    public static AprilTagProcessor aprilTag;
    public static VisionPortal visionPortal;

    /**
     * Initializes the parts on the robot when called (motors, servos, vision, sensors, etc.).<br/>
     * Due to the IMU initialization, this may take around 1-3 seconds to fully initialize. Do this only in init.
     * @param map The hardware map (usually called hardwareMap)
     */
    public static void init(HardwareMap map) {
        // Get the mecanum wheel motors
        leftFront = map.get(DcMotorEx.class, "leftFront");
        rightFront = map.get(DcMotorEx.class, "rightFront");
        leftBack = map.get(DcMotorEx.class, "leftBack");
        rightBack = map.get(DcMotorEx.class, "rightBack");

        // Ensure power is 0
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Stop and reset the encoders
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Set zero power mode to BRAKE
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set direction
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // Use encoders (1440 ticks = 1 rotation, needs testing)
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Now, define the other components
        intake = map.get(DcMotorEx.class, "intake");
        belt = map.get(CRServo.class, "belt");
        kick = map.get(CRServo.class, "kick");
        flywheelLeft = map.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = map.get(DcMotorEx.class, "flywheelRight");

        // Settings for intake
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Settings for flywheels
        flywheelLeft.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorEx.Direction.REVERSE);

        flywheelLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Set orientation and position of webcam
        Position camPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles camOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        // Create and build AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(camPosition, camOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation(3);

        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(map.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .addProcessor(aprilTag)
                .setAutoStartStreamOnBuild(true)
                .build();
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
}
