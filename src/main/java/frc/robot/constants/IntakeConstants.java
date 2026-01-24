package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    public static final double maxVBus = 0;
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
    public static NeutralModeValue neutralMode;
    public static InvertedValue inverted;
    public static double maxCurrent;
    public static double maxPosTicks;
    public static double minPosTicks;

    public static class pidf {

        public static double p;
        public static double i;
        public static double d;
        public static double f;
    }
    ;
}
