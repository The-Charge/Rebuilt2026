package frc.robot.units;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.Logger;

public class ShooterVelocity {

    private double motorRPM;

    private ShooterVelocity(double _motorRots) {
        motorRPM = _motorRots;
    }

    public static ShooterVelocity fromMotorRPM(double rpm) {
        return new ShooterVelocity(rpm);
    }

    public static ShooterVelocity fromMotorVelocity(AngularVelocity vel) {
        if (vel == null) {
            Logger.reportWarning("Cannot create a ShooterVelocity from a null velocity", true);
            return new ShooterVelocity(0);
        }

        return fromMotorRPM(vel.in(Units.RPM));
    }

    public static ShooterVelocity fromMechanismRPM(double rpm) {
        return fromMotorRPM(rpm * ShooterConstants.gearRatio);
    }

    public static ShooterVelocity fromMechanismVelocity(AngularVelocity vel) {
        if (vel == null) {
            Logger.reportWarning("Cannot create a ShooterVelocity from a null velocity", true);
            return new ShooterVelocity(0);
        }

        return fromMechanismRPM(vel.in(Units.RPM));
    }

    public double getAsMotorRPM() {
        return motorRPM;
    }

    public AngularVelocity getAsMotorVelocity() {
        return AngularVelocity.ofBaseUnits(getAsMotorRPM(), Units.RPM);
    }

    public double getAsMechanismRPM() {
        return motorRPM / ShooterConstants.gearRatio;
    }

    public AngularVelocity getAsMechanismVelocity() {
        return AngularVelocity.ofBaseUnits(getAsMechanismRPM(), Units.RPM);
    }

    public ShooterVelocity add(ShooterVelocity b) {
        if (b == null) {
            Logger.reportWarning("Cannot add a null ShooterVelocity", true);
            return fromMotorRPM(getAsMotorRPM()); // return copy of self
        }

        return fromMotorRPM(getAsMotorRPM() + b.getAsMotorRPM());
    }
}
