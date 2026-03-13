package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Time;
import java.util.Optional;

public class IndexerConstants {

    public static final String subsystemName = "Indexer";

    public static class Spindexer {
        public static final int motorID = 17;
        public static final double gearRatio = 48d / 1;

        public static final int maxCurrent = 30;
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = true;
        public static final double maxDutyCycle = 1;
        public static final Optional<Double> nominalVoltage = Optional.empty();
    }

    public static class Exchange {
        public static final int motorID = 16;

        public static final int maxCurrent = 30;
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = false;
        public static final double maxDutyCycle = 1;
        public static final Optional<Double> nominalVoltage = Optional.empty();
    }

    public static final double exchangeVoltage = 0.4 * 12;
    public static final double spindexerVoltage = 12;
    public static final double spindexerAmplitudeVoltage = 0.1 * 12;
    public static final Time spindexerPeriod = Seconds.of(1);
}
