package frc.robot.units;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.TurretConstants;
import frc.robot.utils.Logger;

// Converts between angle of Motor and angle of turret Mechanism; using constant TurretConstants.motorRotsPerMechRots
// Also has a wrap and checks for validity of angle
public class TurretAngle {

    private double motorRots;

    private TurretAngle(double _motorRots) {
        motorRots = _motorRots;
    }

    public static TurretAngle fromMotorRotations(double rotations) {
        return new TurretAngle(rotations);
    }

    public static TurretAngle fromMotorAngle(Angle angle) {
        return fromMotorRotations(angle.in(Rotations));
    }

    public static TurretAngle fromMechanismRotations(double mechRotations) {
        return TurretAngle.fromMotorRotations(mechRotations * TurretConstants.motorRotsPerMechRots);
    }

    public static TurretAngle fromMechanismAngle(Angle mechAngle) {
        return fromMechanismRotations(mechAngle.in(Rotations));
    }

    public double asMotorRotations() {
        return motorRots;
    }

    public Angle asMotorAngle() {
        return Rotations.of(asMotorRotations());
    }

    public double asMechanismRotations() {
        return asMotorRotations() * TurretConstants.mechRotsPerMotorRot;
    }

    public Angle asMechanismAngle() {
        return Rotations.of(asMechanismRotations());
    }

    public TurretAngle wrap() {
        final double centerMechRots = TurretConstants.calibrationEndPos.asMechanismRotations();
        return fromMechanismRotations(((((asMechanismRotations() - centerMechRots) % 1) + 1) % 1) + centerMechRots);
        // Examples: 3.2 -> 0.2 -> 1.2 -> 0.2
        //          -3.2 -> -0.2 -> 0.8 -> 0.8
    }

    // Checks if between bounds
    public boolean isLegal() {
        return asMotorRotations() >= TurretConstants.minLegalAngle.asMotorRotations()
                && asMotorRotations() <= TurretConstants.maxLegalAngle.asMotorRotations();
    }

    public TurretAngle add(TurretAngle b) {
        if (b == null) {
            Logger.reportWarning("Cannot add a null TurretAngle", true);
            return fromMotorRotations(asMotorRotations()); // return copy of self
        }

        return fromMotorRotations(asMotorRotations() + b.asMotorRotations());
    }
}
