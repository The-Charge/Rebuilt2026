package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

public class SwerveConstants {

    private SwerveConstants() {}

    public static final String subsystemName = "Swerve";

    public static final LinearVelocity maxTranslationVel = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity maxAngularVel = RotationsPerSecond.of(1.5);
    public static final LinearVelocity deadbandTranslationVel = MetersPerSecond.of(0.01);
    public static final AngularVelocity deadbandAngularVel = RotationsPerSecond.of(0.01);

    public static final double joystickDeadband = 0.1;
    public static final int joystickExponent = 3;

    public static final PIDConstants pathplannerTranslationPID = new PIDConstants(10, 0, 0);
    public static final PIDConstants pathplannerRotationPID = new PIDConstants(7, 0, 0);
    public static final PIDConstants headingPID = new PIDConstants(10, 0, 0);
}
