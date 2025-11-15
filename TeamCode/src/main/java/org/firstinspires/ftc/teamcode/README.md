## TeamCode Module

Welcome!

TeamCode is where you will put all related code files for controlling the robot. This will be the
only place to store these types of code files.

## Committing to the remote repository

The remote repository consists of two main branches (independent lines of development):

- **master**: This syncs with the base FTCRobotController and is only used to receive necessary
  package upgrades from the base repository. **This should not be committed to.**
- **competition-code**: **This is where you will commit new code.** We will use a system of *pull
  requests* and approvals to update and merge code. Make sure your pull requests go towards this
  branch.

## Creating your own OpModes

There are two main types of OpModes and movement programs: RobotAuto and RobotTeleOp: one is for the
autonomous period and one is for the tele-op period. Both files have special annotations that are
needed for functionality:

- RobotAuto files uses a @Autonomous(name, group) annotation.
- RobotTeleOp files use a @TeleOp(name, group) annotation.

Enabling and disabling OpModes is done with the @Disabled annotations. If this appears, this file
will not be shown as an available OpMode. Comment out or delete this annotation to enable.

## Getting sample OpModes

Sample OpModes are located at
FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external.samples/externalhardware.
Read the README inside that folder for more information.

## Naming

The class names here will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Robot:      This contains an entire OpMode, either Auto or TeleOp. An OpMode is a class containing
            all the code needed to drive the robot during the autonomous and teleop stages.

RobotMap:   There should only be one RobotMap in a project at any given time. A RobotMap
            helps initialize all the parts of the robot, including all motors, sensors, and servos.

Methods:    This is a file that contains a class containing methods used for numerous robot actions.
            You will usually see these files containing methods that encapsulate some form of
            movement (like turning or strafing) or a particular action (like picking up an artifact
            or extending an arm).

After the prefix, other conventions will apply:

* Robot class names are constructed as:     Robot - Mode - Alliance - Location

For instance, an Autonomous for the blue alliance when the robot is located in the small
launch zone, left side, may be called `RobotAutoBlueSmallLeft.java`.  
However, a TeleOp for red alliance in the large launch zone, against the wall, right-side, may be
named `RobotAutoRedLargeRightWall.java`.  
Remember to be consistent with the naming!

## That's it!

Check out the existing files here and understand how everything works, or start programming!
