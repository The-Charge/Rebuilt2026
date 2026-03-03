package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class TurretConstants { // split into shooter, spinner, and hood
    public static final String subsystemName = "Turret";

    public static final int hoodChannel = 17;

    public static final int motorID = 15;
    public static final int maxCurrent = 10;
    public static final IdleMode idleMode = IdleMode.kCoast;
    public static final boolean inverted = false;
    public static final double maxDutyCycle = .10;
    public static final Optional<Double> nominalVoltage = Optional.empty();

    public static final boolean forwardHardLimitEnabled = true;
    public static final Optional<Double> forwardHardLimitResetRots = Optional.empty();
    public static final boolean reverseHardLimitEnabled = true;
    public static final Optional<Double> reverseHardLimitResetRots = Optional.empty();

    public static final double kP = 0.3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final Optional<Voltage> kStaticG = Optional.empty();
    public static final Optional<Voltage> kCos = Optional.empty();
    public static final Optional<Voltage> kS = Optional.empty();
    public static final Optional<Voltage> kV = Optional.empty();
    public static final Optional<Voltage> kA = Optional.empty();
    public static final Optional<Double> iZone = Optional.empty();

    public static final double ticksPerRotation = 30.5; // We don't know why this is the case, it just is.
    public static final double rotationsPerTick = 1 / ticksPerRotation;

    public static final Angle centerAngle = Degrees.of(69.420);
    public static final Angle epsilon = Degrees.of(30);
    public static final Angle minAngle = centerAngle.minus(epsilon);
    public static final Angle maxAngle = centerAngle.plus(epsilon);
    public static final double calibrationSpeed = 0.1;
    public static final double manualSpeed = 0.1;

    public static final int limitSwitchChannel = 9;
}
