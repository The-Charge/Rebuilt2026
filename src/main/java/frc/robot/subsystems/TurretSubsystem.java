package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.TurretConstants.IllegalAngle;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

// shoot angle, turret
public class TurretSubsystem extends SubsystemBase {
    // private final SparkFlex shooter;
    // private final Servo hood;

    private Angle offset = Radians.of(Math.PI);
    private Optional<Angle> targetAngle;
    private final SparkMax turretMotor;

    public TurretSubsystem() {
        turretMotor = new SparkMax(TurretConstants.motorID, MotorType.kBrushless); // port number under IndexerConstants
        SparkMaxConfig turretConfig = new SparkMaxConfig();

        SparkUtils.configureBasicSettings(
                turretConfig,
                TurretConstants.maxCurrent,
                TurretConstants.idleMode,
                TurretConstants.inverted,
                TurretConstants.maxDutyCycle,
                TurretConstants.nominalVoltage);
        SparkUtils.configureClosedLoopSettings(
                turretConfig,
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.kStaticG,
                TurretConstants.kCos,
                TurretConstants.kS,
                TurretConstants.kV,
                TurretConstants.kA);

        if (turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure turret motor");
            // Alerts.spindexerConfigFail.set(true);
        }

        targetAngle = Optional.empty();

        // SmartDashboard.putNumber("gearRatio", 27);

        // shooter = new SparkFlex(TurretConstants.shooterId, MotorType.kBrushless);
        // hood = new Servo(TurretConstants.hoodChannel);
    }

    public void setTurretAngle(Angle angle) {
        if (angle == null) {
            Logger.reportWarning("Cannot set turret angle to a null angle", true);
            return;
        }

        targetAngle = Optional.of(angle);

        if (IllegalAngle.isIllegal(angle)) {
            return;
        }

        angle = IllegalAngle.toContinuousAngle(angle);

        double request = angle.plus(offset).in(Rotations) * TurretConstants.ticksPerRotation;
        // double request = angle.in(Rotations) * SmartDashboard.getNumber("gearRatio", 27);
        turretMotor.getClosedLoopController().setSetpoint(request, ControlType.kPosition);
    }

    public void stop() {
        turretMotor.set(0);
        targetAngle = Optional.empty();
    }

    public Angle getTurretRawAngle() {
        // return new Rotation2d(turret.getEncoder().getPosition() * TurretConstants.Spin.radiansPerTick).plus(offset);
        return Rotations.of(turretMotor.getEncoder().getPosition());
    }

    public Optional<Angle> getTargetAngle() {
        return targetAngle;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(TurretConstants.subsystemName, this);

        Logger.logSparkMotor(TurretConstants.subsystemName, "motor", turretMotor);

        Alerts.turretDisconnected.set(!SparkUtils.isConnected(turretMotor));
        Alerts.turretOverheating.set(turretMotor.getMotorTemperature() >= 80);
        Alerts.turretWarnings.set(SparkUtils.hasCriticalWarnings(turretMotor.getWarnings()));
        Alerts.turretFaults.set(SparkUtils.hasCriticalFaults(turretMotor.getFaults()));
    }
}
