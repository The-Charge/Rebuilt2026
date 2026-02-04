package frc.robot.constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.units.ClimberPosition;
import java.util.Optional;

public class ClimberConstants {

    public static final String subsystemName = "Climber";

    public static final int motorID = 5;
    public static final float mechanismInchesPerMotorRotation = 1;

    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static final double maxCurrent = 60;
    public static final double maxDutyCycle = 1;
    public static final Optional<Double> maxVoltage = Optional.of(12d);
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final Optional<Double> kG = Optional.empty();
    public static final Optional<GravityTypeValue> kGType = Optional.empty();

    public static final ClimberPosition upPosition = ClimberPosition.fromMechanismInches(90);
    public static final ClimberPosition downPosition = ClimberPosition.fromMechanismInches(0);
    public static final ClimberPosition targetTolerance = ClimberPosition.fromMechanismInches(0.1);
}
