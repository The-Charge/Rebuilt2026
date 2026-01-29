package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.units.SpindexerVelocity;
import java.util.Optional;

public class IndexerConstants {

    public static final String subsystemName = "Indexer";

    public static class Spindexer {
        public static final int motorID = 5;
        public static final double gearRatio = 40d / 1;

        public static final double maxCurrent = 30;
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
        public static final double maxDutyCycle = 1;
        public static final double maxVoltage = 16;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;

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
