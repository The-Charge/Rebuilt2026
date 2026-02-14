package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import java.util.Optional;

public class TurretConstants { // split into shooter, spinner, and hood
    public static final int hoodChannel = 17;

    public static double shooterAcceptableAngle;

    public class Spin {
        public static final int motorID = 9;
        public static final int maxCurrent = 10;
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = false;
        public static final double maxDutyCycle = 1.0;
        public static final Optional<Double> nominalVoltage = Optional.empty();
        public static final double kP = 0.3;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final Optional<Double> kStaticG = Optional.empty();
        public static final Optional<Double> kCos = Optional.empty();
        public static final Optional<Double> kS = Optional.empty();
        public static final Optional<Double> kV = Optional.empty();
        public static final Optional<Double> kA = Optional.empty();

        public static final double ticksPerRotation = 3 * 3 * 4; // 3 3 4 test motor
        public static final double rotationsPerTick = 1 / ticksPerRotation;
    }

    public static final String subsystemName = "Turret";
}
