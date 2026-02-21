package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.TurretConstants;

public class IllegalTurretAngle {

    private IllegalTurretAngle() {}

    // Returns if the give angle cannot be pointed at by the turret
    public static boolean isIllegal(Angle theta) {
        theta = wrap(theta);
        return theta.gte(TurretConstants.minAngle) && theta.lte(TurretConstants.maxAngle);
    }

    // Moves an angle into [0, 360)
    public static Angle wrap(Angle theta) {
        return Degrees.of(((theta.in(Degrees) % 360) + 360) % 360); // Proper modulus for negative values
    }

    public static Angle toContinuousAngle(Angle theta) {
        return wrap(theta.minus(TurretConstants.maxAngle)).plus(TurretConstants.maxAngle);
    }
}
