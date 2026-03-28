package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.units.TurretAngle;
import java.util.Optional;

public class TurretConstants {
    public static final String subsystemName = "Turret";

    public static class Motor {
        public static final int motorID = 15;
        public static final String motorName = "turretMotor";

        public static final Current maxCurrent = Amps.of(10);
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = true;
        public static final double maxDutyCycle = 1;
        public static final Optional<Voltage> nominalVoltage = Optional.empty();

        public static final boolean forwardHardLimitEnabled = false;
        public static final Optional<Angle> forwardHardLimitResetRots = Optional.empty();
        public static final boolean reverseHardLimitEnabled = false;
        public static final Optional<Angle> reverseHardLimitResetRots = Optional.empty();

        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final Optional<Double> iZone = Optional.empty();
        public static final double kD = 0.1;
        public static final Optional<Voltage> kStaticG = Optional.empty();
        public static final Optional<Voltage> kCos = Optional.empty();
        public static final Optional<Double> kS = Optional.empty();
        public static final Optional<Double> kV = Optional.empty();
        public static final Optional<Double> kA = Optional.empty();
        public static final Optional<Time> rampTime = Optional.of(Seconds.of(0.1));

        public static final AngularAcceleration maxAccel =
                RotationsPerSecondPerSecond.of(16666.66 / 60); // Natural max acceleration
        public static final Optional<AngularVelocity> cruiseVel = Optional.of(RPM.of(11000));
        public static final Optional<Angle> allowedError = Optional.empty();

        public static final double motorRotsPerMechRots = (46.713913 - -19.690401) * 2;

        public static final double manualSpeed = 0.1;

        public static final double calibrationSpeed = -0.2;
        public static final TurretAngle calibrationEndPos = TurretAngle.fromMotorRotations(-39.475796);
        public static final Time calibrationEndDelay = Seconds.of(0.2);
        public static final Current calibrationThresholdCurrent = Amps.of(20);
        public static final AngularVelocity calibrationThresholdVel = RPM.of(1);
    }

    public static final TurretAngle minLegalAngle = TurretAngle.fromMechanismRotations(-0.25);
    public static final TurretAngle maxLegalAngle = TurretAngle.fromMechanismRotations(0.5);

    public static Translation2d turretCenterOffset =
            new Translation2d(Inches.of(-6), Inches.of(-6)); // vector from center of robot to center of turret

    public static final TurretAngle targetTolerance = TurretAngle.fromMechanismAngle(Degrees.of(5));
}
