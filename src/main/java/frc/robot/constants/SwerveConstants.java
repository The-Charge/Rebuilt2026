package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class SwerveConstants {

    private SwerveConstants() {}

    public static final LinearVelocity maxTranslationVel = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity maxAngularVel = RotationsPerSecond.of(1.5);

    public static final double joystickDeadband = 0.1;
    public static final int joystickExponent = 3;
}
