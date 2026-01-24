package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    public static int RollerMotorId = 0;
    public static int PivotMotorId = 0;

    // Roller motor speed when the intake is out and collecting
    public static double RollerSpeed = 1.00;

    // Pivot motor speed when deploying and retracting the pivot
    public static double PivotSpeed = 1.00;

    public static double DeployedPosition = 1.234567;
    public static double RetractedPosition = 7.654321;

    public static double RollerVelocity = 123.456;

    public static double StateTolerance = 1.23456;

    public static double maxVBus = 1.00;
    public static NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static double maxCurrent = 35;
    public static double minPosTicks = 0;
    public static double maxPosTicks = 123;

    public static class pidf {
        public static double p = 0.3;
        public static double i = 0;
        public static double d = 0.02;
        public static double f = 0;
    }
}
