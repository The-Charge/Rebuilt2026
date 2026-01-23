package frc.robot.constants;

import java.util.Optional;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexConstants {

    public static final int spindexerMotorID = 67;

    public static final double maxCurrent = 30;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
    public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static final double maxDutyCycle = 1;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    //TODO: indexer units
    public static final Optional<Double> forwardSoftLimit = Optional.empty();
    public static final Optional<Double> reverseSoftLimit = Optional.empty();
    public static final double maxVoltage = 16;
    public static final boolean forwardHardLimitEnabled = false;
    public static final Optional<Double> forwardHardLimitResetValue = Optional.empty();
    public static final boolean reverseHardLimitEnabled = false;
    public static final Optional<Double> reverseHardLimitResetValue = Optional.empty();
    public static final double spindexerMotorVelocity = 0;
}