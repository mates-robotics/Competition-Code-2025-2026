package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.resources.RobotMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="DECODE Teleop", group="Robot")
public class RobotTeleop extends LinearOpMode {

    // 16 inches

    // Fixed speed; all values will be clipped by these values
    private final double MAX_TPS_DRIVE = 300;       // Max speed for driving, in TPS
    private final double MAX_FLYWHEEL_SPEED = 500;  // Max speed for flywheel launcher wheels

    // Variables for calibration, during shooting
    private final double DRIVE_GAIN = 0.02;         // Forward speed control change for calibrating. 50% power at 25 in. error (.50 / 25)
    private final double STRAFE_GAIN = 0.015;       // Strafe speed control change for calibrating. 37% power at 25º yaw error (.35 / 25)
    private final double TURN_GAIN = 0.01;          // Turn speed control change for calibrating. 25% power at 25º bearing error (.25 / 25)

    private final double DESIRED_RANGE = 63;        // The desired distance for the robot to be located at relative to the AprilTag.
    private final double DESIRED_RANGE_ERROR = 13;  // Amount of error tolerated for the desired range.
    private final double GOAL_ADDED_RANGE = 16;     // Inches added to final distance to account for landing the ball most efficiently

    private boolean IS_CALIBRATING = false;         // If calibrating, this is true

    // AprilTag detection tags
    private int teamTag = 0;

//    private int motifTag = 0;
    private boolean ALL_TAGS_PRESENT = false;

    // Tag reset - in case we catastrophically set the team tag to the other team...
    private int TEAM_TAG_RESET_COUNT = 0;

    // Last pressed buttons on specific controller
    private boolean lastSquareG1 = false;
    private boolean lastSquareG2 = false;

    // Variables for telemetry
    private AprilTagDetection currentDetection = null;


    @Override
    public void runOpMode()
    {
        // Initialize robot map
        RobotMap.init(hardwareMap);

        // Set exposure
        // Setting the exposure to reduce motion blur
        while (!isStopRequested() && (RobotMap.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }
        if (!isStopRequested())
        {
            // Edit the exposure
            ExposureControl exposureControl = RobotMap.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }

            exposureControl.setExposure((long)6, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = RobotMap.visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            sleep(20);
        }

        // Just a test
        telemetry.addData("Robot status", "Ready");
        telemetry.update();

        waitForStart();

        // Abbreviated mapped buttons (PS4)
        //   L2                               R2   (triggers)
        //   L1                               R1   (bumpers)
        //      U                           ▲
        //   L     R                     ■     ●   (D-pad and shape buttons)
        //      D                           x
        //             LS          RS              (sticks)

        // NOTE: LS and RS joystick y-values are negative when pushed forward, so negation of this value is common

        while (opModeIsActive())
        {
            // ● for intake
            if (gamepad1.circle || gamepad2.circle)
            {
                RobotMap.intake.setPower(1);
            }

            // x manual movement of belt
            // TODO
            else if (gamepad1.cross || gamepad2.cross)
            {
                RobotMap.belt.setPower(1);
            }

            // ▲ manual shooting if necessary
            // TODO
            else if (gamepad1.triangle || gamepad2.triangle)
            {
                RobotMap.flywheelLeft.setVelocity(MAX_FLYWHEEL_SPEED);
                RobotMap.flywheelRight.setVelocity(MAX_FLYWHEEL_SPEED);
            }

            // L1 and R1 calibration and automatic shooting
            // Much recommended as it removes human error
            // Only works when pressed
            else if (gamepad1.left_bumper || gamepad2.left_bumper)
            {
                // Lots of logic, so offloaded into a method
                calibrate();
            }

            // When bumper is released, stop calibration
            if (!gamepad1.left_bumper && !gamepad2.left_bumper)
            {
                // Stop calibration
                IS_CALIBRATING = false;
            }

            // ■ x3 override team tag information (backup, catastrophic)
            // Useful if findTags saved the wrong team ID
            // Check for a press; that means that last time this loop ran this button shouldn't have been down
            if ((gamepad1.square && !lastSquareG1) || (gamepad2.square && !lastSquareG2))
            {
                // Add to team tag reset count, check for 3rd press
                // Just fancy one-liner code
                if (++TEAM_TAG_RESET_COUNT == 3)
                {
                    // Reset the team tag ID
                    teamTag = 0;
                    // Reset counter
                    TEAM_TAG_RESET_COUNT = 0;
                }
            }

            // Use LS and RS to move the robot
            // Each side controls each side of the robot
            // For instance, left LS and left RS strafes left
            // while forward LS and backward RS turns to the right
            drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y);

            // Set last press variables
            updatePressVariables();

            telemetry();
        }
    }

    /**
     * Moves the robot, given the four values from the gamepad
     * @param lx The x-value of the left joystick
     * @param ly The NEGATED y-value of the left joystick
     * @param rx The x-value of the right joystick
     * @param ry The NEGATED y-value of the right joystick
     */
    public void drive(double lx, double ly, double rx, double ry)
    {
        // If the bumper is held and it is calibrating, stop
        if (IS_CALIBRATING) return;

        // Variables stand for location (lf = leftFront), then p for power
        // The speeds are determined based on forward direction and mecanum wheel direction for strafing
        double lfp = ly + lx;
        double lbp = ly - lx;
        double rfp = ry - rx;
        double rbp = ry + ry;

        driveValues(lfp, lbp, rfp, rbp);
    }

    private void driveValues(double lfp, double lbp, double rfp, double rbp)
    {
        // Since some values may exceed absolute value 1, we need to make sure that doesn't happen
        // If a speed exceeds 1, we will divide all the powers by the maximum speed over 1
        double max = 1.0;

        // Check through all speeds and find the max speed
        // Obviously, absolute value, since it can have less that -1 speed as well.
        max = Math.max(max, Math.abs(lfp));
        max = Math.max(max, Math.abs(lbp));
        max = Math.max(max, Math.abs(rfp));
        max = Math.max(max, Math.abs(rbp));

        // Now, set the powers, relative to the max
        RobotMap.leftFront.setVelocity(MAX_TPS_DRIVE * (lfp / max));
        RobotMap.rightFront.setVelocity(MAX_TPS_DRIVE * (rfp / max));
        RobotMap.leftBack.setVelocity(MAX_TPS_DRIVE * (lbp / max));
        RobotMap.rightBack.setVelocity(MAX_TPS_DRIVE * (rbp / max));
    }

    /**
     * Finds the location of two specific AprilTags and stores them into variables, if not already present:
     * <ul>
     *     <li>teamTag: the tag corresponding to our alliance's goal</li>
     *     <li>motifTag: the tag corresponding to the motif</li>
     * </ul>
     */
    public void findTags()
    {
        List<AprilTagDetection> detections = RobotMap.aprilTag.getDetections();
        for (AprilTagDetection detection: detections)
        {
            // Blue AprilTag ID is 20, Red AprilTag ID is 24
            if (teamTag == 0 && (detection.metadata.id == 20 || detection.metadata.id == 24))
            {
                // We found the AprilTag, set ID to that
                teamTag = detection.metadata.id;
                // We don't have to look for any more tags
            }
//            // Motif AprilTag IDs are 21, 22, 23
//            else if (motifTag == null && detection.metadata.id >= 21 && detection.metadata.id <= 23)
//            {
//                // We found the AprilTag, calibrate to that
//                motifTag = detection;
//                // We don't have to look for any more tags
//            }
        }

        // If both tags have been recorded, set tags to true
        if (teamTag != 0 /* && motifTag != 0 */) ALL_TAGS_PRESENT = true;
    }

    /**
     * Attempts to detect a valid team tag, and moves the robot accordingly to align with the tag.<br/>
     * The controller will vibrate if the following actions occur:
     * <ul>
     *     <li>One short rumble (~0.3 sec) - cannot find AprilTag</li>
     *     <li>One long rumble (~0.8 sec) - calibrated successfully</li>
     * </ul>
     */
    public void calibrate()
    {
        currentDetection = null;
        // Get AprilTag detections, search for teamTag ID
        List<AprilTagDetection> detections = RobotMap.aprilTag.getDetections();
        for (AprilTagDetection detection: detections)
        {
            // Find the tag for our goal
            if (detection.metadata.id == teamTag)
            {
                // Found, we don't need to iterate over the rest
                currentDetection = detection;
                break;
            }
        }

        // AprilTag not detected, fire three blips
        if (currentDetection == null)
        {
            gamepad1.rumble(300);
            gamepad2.rumble(300);
            return;
        }

        IS_CALIBRATING = true;

        // Now, calibrate from the given AprilTag and calculate the error
        // Range error - distance
        // Bearing error - viewing angle relative to tag
        // Yaw error - angle off center relative to tag
        double rangeError = currentDetection.ftcPose.range - DESIRED_RANGE;
        double bearingError = currentDetection.ftcPose.bearing;
        double yawError = currentDetection.ftcPose.yaw;

        // Clip errors by max drive speed
        rangeError = Range.clip(rangeError * DRIVE_GAIN, -MAX_TPS_DRIVE, MAX_TPS_DRIVE);
        bearingError = Range.clip(bearingError * TURN_GAIN, -MAX_TPS_DRIVE, MAX_TPS_DRIVE);
        yawError = Range.clip(-yawError * STRAFE_GAIN, -MAX_TPS_DRIVE, MAX_TPS_DRIVE);

        // Drive using the values
        driveValues(
                rangeError - yawError - bearingError,
                rangeError + yawError - bearingError,
                rangeError - yawError + bearingError,
                rangeError + yawError + bearingError
        );
    }

    /**
     * Prints out necessary telemetry per run
     */
    public void telemetry()
    {

    }

    public void updatePressVariables()
    {
        lastSquareG1 = gamepad1.square;
        lastSquareG2 = gamepad2.square;
    }
}
