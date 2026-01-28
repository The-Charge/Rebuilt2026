package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.util.Units;

public abstract class SwerveConstants {

    public static final double ROBOT_MASS = (110) * 0.453592; // 32lbs * kg per pound
    // public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED =
            Units.feetToMeters(14.5); // Maximum speed of the robot in meters per second, used to limit acceleration.

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final boolean useChassisVelocityCorrection = false;
    public static final double chassisVelocityCorrection = 0.2;

    public static final boolean useCosineCompensator = true;

    public static final boolean useAngularCompensationTeleop = true;
    public static final boolean useAngularCompensationAuton = true;
    public static final double angularCompensation = -0.1;

    public static final PIDConstants pathPlannerTranslationPID = new PIDConstants(5, 0, 0);
    public static final PIDConstants pathPlannerRotationPID = new PIDConstants(5, 0, 0);
    public static final double autonVisionTime = 0; // number of seconds vision is enabled for in auton

    // Joystick Deadband
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double LEFT_X_DEADBAND = 0.2; // left x is pretty sensitive, so increased deadband
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double RIGHT_Y_DEADBAND = 0.2;
    public static final double TURN_CONSTANT = 6;
    public static final double DRIVE_SPEED = 1; // multiplies translation and rotation, 1 is 100%
    public static final double TRIGGER_DEADBAND = 0.1;

    public static final double reefAcceptableDist = 210; // millimeters
    public static final double reefAlignSpeed = 1; // meters/sec
    public static final double sourceAcceptableDist = 170; // millimeters
    public static final double sourceAlignSpeed = 1; // meters/sec
    public static final double alignTimeout = 2; // seconds
    public static final PIDConstants alignPID = new PIDConstants(0.003, 0, 0);

    public static final double branchAlignSpeed = 0.24; // m/s
}
