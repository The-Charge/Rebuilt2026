package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.units.ClimberPosition;
import java.util.Optional;

public class ClimberConstants {

    public static final String subsystemName = "Climber";

    public static class Motor {
        public static final int motorID = 3;
        public static final String motorName = "climbMotor";
        public static final double mechanismInchesPerMotorRotation = 1; // TODO: climber conversion factor

        public static final Current maxCurrent = Amps.of(40);
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
        public static final double maxDutyCycle = 1;
        public static final Optional<Voltage> maxVoltage = Optional.empty();

        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final Optional<Double> kG = Optional.empty();
        public static final Optional<GravityTypeValue> kGType = Optional.empty();
        public static final Optional<Double> kS = Optional.empty();
        public static final Optional<StaticFeedforwardSignValue> kSSign = Optional.empty();
        public static final Optional<Double> kV = Optional.empty();
        public static final Optional<Double> kA = Optional.empty();

        public static final ClimberPosition upPosition = ClimberPosition.fromMotorRotations(415.21);
        public static final ClimberPosition climbPosition = ClimberPosition.fromMotorRotations(145.88);
        public static final ClimberPosition downPosition = ClimberPosition.fromMotorRotations(0);

        public static final ClimberPosition targetTolerance = ClimberPosition.fromMotorRotations(5);
        public static final double manualSpoolSpeed = 0.2;
    }
}
