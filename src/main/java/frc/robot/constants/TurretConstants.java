package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class TurretConstants { // split into shooter, spinner, and hood
    public static final int spinMotorId = -1;
    public static final int hoodChannel = -1;
    public static final int turretId = -1;
    public static final double ticksPerRadian = -1;
    public static final double radiansPerTick = 1 / ticksPerRadian;
    public static final double kP = -1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final int shooterMotorId = -1;
    public static final IdleMode idleMode = IdleMode.fromId(-1);
    public static final int currentLimit = -1;
    public static final boolean inverted = false;

    public static final double rangeThreshold = -1;
}
