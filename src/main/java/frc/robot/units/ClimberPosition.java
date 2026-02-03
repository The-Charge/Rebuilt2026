package frc.robot.units;

public class ClimberPosition {
    private double motorRots;

    private static double inchesPerRotation = 4.20;

    private ClimberPosition(double _motorRotation) {
        motorRots = _motorRotation;
    }

    public static ClimberPosition fromMotorRotations(double rotations) {
        return new ClimberPosition(rotations);
    }

    public static ClimberPosition fromMechanismInches(double inches) {
        return ClimberPosition.fromMotorRotations(inches / inchesPerRotation);
    }

    public double toMotorRots() {
        return motorRots;
    }

    public double toMotorDegrees() {
        return motorRots * inchesPerRotation;
    }
}
