package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class IntakeConstants {
    public static final String subsystemName = "Intake";

    public static class Roller {
        public static final int motorID = 14;
        public static final String motorName = "rollerMotor";

        public static final Current currentLimit = Amps.of(40);
        public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
        public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
        public static final double maxDutyCycle = 1.00;
        public static final Optional<Voltage> maxVoltage = Optional.empty();

        public static final double intakeVoltage = 12;
        public static final double shootVoltage = 12;
    }

    public static class LeftDeployer {
        public static final int port = 1;
        public static final String servoName = "leftDeployerServo";

        public static final double deployedPosition = 1;
    }

    public static class RightDeployer {
        public static final int port = 3;
        public static final String servoName = "rightDeployerServo";

        public static final double deployedPosition = 0.25;
    }
}
