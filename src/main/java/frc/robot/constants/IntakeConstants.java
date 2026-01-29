package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    public static int RollerMotorId = 9;
    public static int PivotMotorId = 5;

    public static double DeployedPosition = 2000;
    public static double RetractedPosition = 0;

    public static double StateTolerance = 500;
    public static int currentLimit = 20;

    public static class PivotConfig {
        public static double maxVBus = 1.00;
        public static NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static InvertedValue inverted = InvertedValue.Clockwise_Positive;
        public static double maxCurrent = 35;
        public static double minPosTicks = 0;
        public static double maxPosTicks = 1;

        public static class pidf {
            public static double p = 0.8;
            public static double i = 0.3;
            public static double d = 0.02;
            public static double f = 0.2;
        }
    }
}
