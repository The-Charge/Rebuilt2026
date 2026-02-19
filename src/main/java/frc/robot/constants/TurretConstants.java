package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Angle;
import java.util.Optional;

public class TurretConstants { // split into shooter, spinner, and hood
    public static final String subsystemName = "Turret";

    public static final int hoodChannel = 17;

    public static final int motorID = 9;
    public static final int maxCurrent = 10;
    public static final IdleMode idleMode = IdleMode.kCoast;
    public static final boolean inverted = false;
    public static final double maxDutyCycle = .10;
    public static final Optional<Double> nominalVoltage = Optional.empty();
    public static final double calibrationSpeed = 0.1;

    public static final double kP = 0.3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final Optional<Double> kStaticG = Optional.empty();
    public static final Optional<Double> kCos = Optional.empty();
    public static final Optional<Double> kS = Optional.empty();
    public static final Optional<Double> kV = Optional.empty();
    public static final Optional<Double> kA = Optional.empty();

    public static final double ticksPerRotation = 30.5; // We don't know why this is the case, it just is.
    public static final double rotationsPerTick = 1 / ticksPerRotation;

    public static final class IllegalAngle {
        public static final Angle center = Degrees.of(69.420);
        public static final Angle epsilon = Degrees.of(30);
        public static final Angle min = center.minus(epsilon);
        public static final Angle max = center.plus(epsilon);

        // Returns if the give angle cannot be pointed at by the turret
        public static boolean isIllegal(Angle theta) {
            theta = wrap(theta);
            return theta.gte(min) && theta.lte(max);
        }

        // Moves an angle into [0, 360)
        public static Angle wrap(Angle theta) {
            return Degrees.of(((theta.in(Degrees) % 360) + 360) % 360); // Proper modulus for negative values
        }

        public static Angle toContinuousAngle(Angle theta) {
            return wrap(theta.minus(max)).plus(max);
        }
    }
}
