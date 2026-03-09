package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.Optional;

public class IntakeConstants {
    public static final String name = "Intake";

    public static class Roller {
        public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
        public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
        public static final double maxDutyCycle = 1.00;
        public static final Optional<Double> maxVoltage = Optional.empty();
        public static final int currentLimit = 20;
        public static final int motorID = 14;
        public static final double overheatingTemp = 80; // celcius

        public static final double intakeVoltage = 12;
    }

    public static class LeftDeployer {
        public static final int port = 1;

        public static final double deployedPosition = 1;
    }

    public static class RightDeployer {
        public static final int port = 3;

        public static final double deployedPosition = 0.5;
    }
}
