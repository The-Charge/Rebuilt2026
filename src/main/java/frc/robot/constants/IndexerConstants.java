package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.units.SpindexerVelocity;
import java.util.Optional;

public class IndexerConstants {

    public static final String subsystemName = "Indexer";

    public static class Spindexer {
        public static final int motorID = 5;
        public static final double gearRatio = 40d / 1;

        public static final int maxCurrent = 20;
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = false;
        public static final double maxDutyCycle = 1;
        public static final Optional<Double> nominalVoltage = Optional.empty();
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final Optional<Double> kStaticG = Optional.empty();
        public static final Optional<Double> kCos = Optional.empty();
        public static final Optional<Double> kS = Optional.empty();
        public static final Optional<Double> kV = Optional.empty();
        public static final Optional<Double> kA = Optional.empty();

        public static final SpindexerVelocity targetTolerance = SpindexerVelocity.fromMotorRPM(20);
    }

    public static class Gate {
        public static final int motorID = 9;

        public static final int maxCurrent = 20;
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = false;
        public static final double maxDutyCycle = 1;
        public static final Optional<Double> nominalVoltage = Optional.empty();
    }

    public static final SpindexerVelocity spindexerMotorVelocity = SpindexerVelocity.fromMechanismRPM(3000);
    public static final double gateToShooterVoltage = 12;
}
