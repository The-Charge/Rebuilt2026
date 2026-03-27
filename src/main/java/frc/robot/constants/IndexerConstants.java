package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class IndexerConstants {

    public static final String subsystemName = "Indexer";

    public static class Spindexer {
        public static final int motorID = 17;
        public static final String motorName = "spindexerMotor";

        public static final Current maxCurrent = Amps.of(30);
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = true;
        public static final double maxDutyCycle = 1;
        public static final Optional<Voltage> nominalVoltage = Optional.empty();

        public static final double shootVoltage = 12;
        public static final double reverseVoltage = -1;
    }

    public static class Exchange {
        public static final int motorID = 16;
        public static final String motorName = "exchangeMotor";

        public static final Current maxCurrent = Amps.of(30);
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = false;
        public static final double maxDutyCycle = 1;
        public static final Optional<Voltage> nominalVoltage = Optional.empty();

        public static final double shootVoltage = 12;
    }
}
