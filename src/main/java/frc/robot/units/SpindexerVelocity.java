package frc.robot.units;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.IndexerConstants;
import frc.robot.utils.Logger;

public class SpindexerVelocity {

    private double motorRPM;

    private SpindexerVelocity(double _motorRots) {
        motorRPM = _motorRots;
    }

    public static SpindexerVelocity fromMotorRPM(double rpm) {
        return new SpindexerVelocity(rpm);
    }

    public static SpindexerVelocity fromMotorVelocity(AngularVelocity vel) {
        if (vel == null) {
            Logger.reportWarning("Cannot create a SpindexerVelocity from a null velocity", true);
            return new SpindexerVelocity(0);
        }

        return fromMotorRPM(vel.abs(Units.RPM));
    }

    public static SpindexerVelocity fromMechanismRPM(double rpm) {
        return fromMotorRPM(rpm * IndexerConstants.Spindexer.gearRatio);
    }

    public static SpindexerVelocity fromMechanismVelocity(AngularVelocity vel) {
        if (vel == null) {
            Logger.reportWarning("Cannot create a SpindexerVelocity from a null velocity", true);
            return new SpindexerVelocity(0);
        }

        return fromMechanismRPM(vel.abs(Units.RPM));
    }

    public double getAsMotorRPM() {
        return motorRPM;
    }

    public AngularVelocity getAsMotorVelocity() {
        return AngularVelocity.ofBaseUnits(getAsMotorRPM(), Units.RPM);
    }

    public double getAsMechanismRPM() {
        return motorRPM / IndexerConstants.Spindexer.gearRatio;
    }

    public AngularVelocity getAsMechanismVelocity() {
        return AngularVelocity.ofBaseUnits(getAsMechanismRPM(), Units.RPM);
    }

    public SpindexerVelocity add(SpindexerVelocity b) {
        if (b == null) {
            Logger.reportWarning("Cannot add a null SpindexerVelocity", true);
            return fromMotorRPM(getAsMotorRPM()); // return copy of self
        }

        return fromMotorRPM(getAsMotorRPM() + b.getAsMotorRPM());
    }
}
