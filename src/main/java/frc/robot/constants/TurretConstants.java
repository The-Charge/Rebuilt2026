package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;

public class TurretConstants { // split into shooter, spinner, and hood
    public static final int spinMotorId = 123;
    public static final int hoodChannel = 17;
    public static final int turretId = 82;
    public static final double ticksPerRadian = 1000;
    public static final double radiansPerTick = 1 / ticksPerRadian;
    public static final double kP = -1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final int shooterMotorId = 813;
    public static final int shooterId = 914;
    public static final IdleMode idleMode = IdleMode.fromId(0);
    public static final int currentLimit = 40;
    public static final boolean inverted = false;
    public static double shooterAcceptableAngle;
    public static PIDController shooterPID = new PIDController(0, 0, 0);
}
