package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

// shoot angle, turret
public class TurretSubsystem extends SubsystemBase {
    // private final SparkFlex shooter;
    // private final Servo hood;

    private Rotation2d offset = new Rotation2d(Math.PI);
    private Optional<Rotation2d> targetAngle;
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

        // shooter = new SparkFlex(TurretConstants.shooterId, MotorType.kBrushless);
        // hood = new Servo(TurretConstants.hoodChannel);
    }

    public void setTurretAngle(Rotation2d angle) {
        if (angle == null) {
            Logger.reportWarning("Cannot set spindexer velocity to a null velocity", true);
            return;
        }

        targetAngle = Optional.of(angle);

        double request = angle.plus(offset).getRotations() * TurretConstants.ticksPerRotation;
        turretMotor.getClosedLoopController().setSetpoint(request, ControlType.kPosition);
    }

    public void stop() {
        turretMotor.set(0);
    }

    public Rotation2d getTurretRawAngle() {
        // return new Rotation2d(turret.getEncoder().getPosition() * TurretConstants.Spin.radiansPerTick).plus(offset);
        return new Rotation2d(turretMotor.getEncoder().getPosition());
    }

    public Optional<Rotation2d> getTargetAngle() {
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
