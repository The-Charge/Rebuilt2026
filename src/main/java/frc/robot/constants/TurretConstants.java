package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.units.TurretAngle;
import java.util.Optional;

public class TurretConstants { // split into shooter, spinner, and hood
    public static final String subsystemName = "Turret";

    // public static final int hoodChannel = 17;

    public static final int forwardLimitChannel = 9;

    public static final int motorID = 15;
    public static final int maxCurrent = 10;
    public static final IdleMode idleMode = IdleMode.kCoast;
    public static final boolean inverted = true;
    public static final double maxDutyCycle = 1;
    public static final Optional<Double> nominalVoltage = Optional.empty();

    public static final boolean forwardHardLimitEnabled = false;
    public static final Optional<Double> forwardHardLimitResetRots = Optional.empty();
    public static final boolean reverseHardLimitEnabled = false;
    public static final Optional<Double> reverseHardLimitResetRots = Optional.empty();

    public static final double kP = 0.3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final Optional<Voltage> kStaticG = Optional.empty();
    public static final Optional<Voltage> kCos = Optional.empty();
    public static final Optional<Voltage> kS = Optional.empty();
    public static final Optional<Voltage> kV = Optional.empty();
    public static final Optional<Voltage> kA = Optional.empty();
    public static final Optional<Double> iZone = Optional.empty();

    public static final double motorRotsPerMechRots = (46.713913 - -19.690401) * 2;
    public static final double mechRotsPerMotorRot = 1 / motorRotsPerMechRots;

    public static final TurretAngle minLegalAngle = TurretAngle.fromMechanismRotations(-0.25);
    public static final TurretAngle maxLegalAngle = TurretAngle.fromMechanismRotations(0.25);

    public static final double manualSpeed = 0.1;

    public static final double calibrationSpeed = -0.05;
    public static final TurretAngle calibrationEndPos = TurretAngle.fromMotorRotations(-29.428341);
    public static final Time calibrationResetDelay = Seconds.of(0.2);
    public static final double calibrationThresholdCurrent = 5; // AMPS

}
