package frc.robot.units;

import frc.robot.constants.ClimberConstants.Motor;
import frc.robot.utils.Logger;

public class ClimberPosition {

    private double motorRots;

    private ClimberPosition(double _motorRots) {
        motorRots = _motorRots;
    }

    public static ClimberPosition fromMotorRotations(double rotations) {
        return new ClimberPosition(rotations);
    }

    public static ClimberPosition fromMechanismInches(double inches) {
        return ClimberPosition.fromMotorRotations(inches / Motor.mechanismInchesPerMotorRotation);
    }

    public double asMotorRotations() {
        return motorRots;
    }

    public double asMechanismInches() {
        return asMotorRotations() * Motor.mechanismInchesPerMotorRotation;
    }

    public ClimberPosition add(ClimberPosition b) {
        if (b == null) {
            Logger.reportWarning("Cannot add a null ClimberPosition", true);
            return fromMotorRotations(asMotorRotations()); // return copy of self
        }

        return fromMotorRotations(asMotorRotations() + b.asMotorRotations());
    }
}
