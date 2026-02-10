package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import java.util.Optional;

public class TurretConstants { // split into shooter, spinner, and hood
    public static final int spinMotorId = -1;
    public static final int turretId = -1;
    public static final double ticksPerRadian = -1;
    public static final double radiansPerTick = 1 / ticksPerRadian;

    public static final int shooterMotorId = -1;
    public static final IdleMode idleMode = IdleMode.kBrake;
    public static final int currentLimit = 20;
    public static final boolean inverted = false;

    public static final double rangeThreshold = -1;
    public static final String subsystemName = "Turret";
    public static final double maxDutyCycle = 1.0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final Optional<Double> nominalVoltage = Optional.empty();
    public static final Optional<Double> kStaticG = Optional.empty();
    public static final Optional<Double> kCos = Optional.empty();
    public static final Optional<Double> kS = Optional.empty();
    public static final Optional<Double> kV = Optional.empty();
    public static final Optional<Double> kA = Optional.empty();
}
