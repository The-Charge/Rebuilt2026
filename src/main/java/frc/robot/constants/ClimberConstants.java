package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.units.ClimberPosition;
import java.util.Optional;

public class ClimberConstants {

    public static final String subsystemName = "Climber";

    public static class Motor {
        public static final int motorID = 3;
        public static final String motorName = "climbMotor";
        public static final double mechanismInchesPerMotorRotation = 1; // TODO: climber conversion factor

        public static final Current maxCurrent = Amps.of(60);
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
        public static final double maxDutyCycle = 1;
        public static final Optional<Voltage> maxVoltage = Optional.empty();

        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final Optional<Double> kG = Optional.empty();
        public static final Optional<GravityTypeValue> kGType = Optional.empty();
        public static final Optional<Double> kS = Optional.empty();
        public static final Optional<StaticFeedforwardSignValue> kSSign = Optional.empty();
        public static final Optional<Double> kV = Optional.empty();
        public static final Optional<Double> kA = Optional.empty();

        public static final ClimberPosition upPosition = ClimberPosition.fromMotorRotations(415.21 / 9);
        public static final ClimberPosition climbPosition = ClimberPosition.fromMotorRotations(145.88 / 9);
        public static final ClimberPosition downPosition = ClimberPosition.fromMotorRotations(0);

        public static final ClimberPosition targetTolerance = ClimberPosition.fromMotorRotations(5 / 9);
        public static final double manualSpoolSpeed = 0.1;
    }

    public static class TowerSensor {
        public static final int sensorID = 32;
        public static final String sensorName = "towerSensor";

        /*
        * From the sensor documentation:
        * The PWF TOF sensor supports three ranging modes. Short mode (default) works the best in bright
          lighting conditions and allows the fastest sample period (24ms), but can only measure 1.3 meters. Long
          mode can measure up to 4 meters in the dark, but may only be able to measure shorter distances
          depending on the lighting conditions and is limited to slower sample periods.
        */
        public static final RangingMode rangingMode = RangingMode.Short;
        public static final Time sampleTime = Milliseconds.of(24); // valid range of 24 - 1000 ms

        public static final double activationMM = 25;
        public static final double activationStdDev = 4;
    }
}
