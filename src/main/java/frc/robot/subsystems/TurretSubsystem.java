package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.TurretConstants;
import frc.robot.units.TurretAngle;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax turretMotor;
    // private final DigitalInput forwardLimit;

    private Optional<TurretAngle> targetAngle;
    private boolean isCalibrated;

    private final Optional<StructPublisher<Pose2d>> turretPosePublisher;
    private final Optional<StructPublisher<Pose2d>> targetTurretPosePublisher;
    private final Optional<StructPublisher<Translation2d>> targetPointPublisher;

    public TurretSubsystem() {
        turretMotor = new SparkMax(TurretConstants.motorID, MotorType.kBrushless);

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
                TurretConstants.kA,
                TurretConstants.iZone);
        SparkUtils.configureHardStops(
                turretConfig,
                TurretConstants.forwardHardLimitEnabled,
                TurretConstants.forwardHardLimitResetRots,
                TurretConstants.reverseHardLimitEnabled,
                TurretConstants.reverseHardLimitResetRots);

        if (turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure turret motor");
            Alerts.turretConfigFail.set(true);
        }

        targetAngle = Optional.empty();
        isCalibrated = false;

        turretPosePublisher = Logger.makeStructPublisher(TurretConstants.subsystemName, "turretPose", Pose2d.struct);
        targetTurretPosePublisher =
                Logger.makeStructPublisher(TurretConstants.subsystemName, "targetTurretPose", Pose2d.struct);
        targetPointPublisher =
                Logger.makeStructPublisher(TurretConstants.subsystemName, "targetPoint", Translation2d.struct);
    }

    public void setTurretAngle(TurretAngle angle) {
        if (angle == null) {
            Logger.reportWarning("Cannot set turret angle to a null angle", true);
            return;
        }

        angle = angle.wrap();
        if (!angle.isLegal()) return;

        targetAngle = Optional.of(angle);

        turretMotor.getClosedLoopController().setSetpoint(angle.asMotorRotations(), ControlType.kPosition);
    }

    public void stopTurret() {
        turretMotor.set(0);
        targetAngle = Optional.empty();
    }

    public boolean isAtCalibrationLimit() {
        return turretMotor.getOutputCurrent() > TurretConstants.calibrationThresholdCurrent
                || Math.abs(turretMotor.getEncoder().getVelocity()) < 1;
    }

    public TurretAngle getCurretAngle() {
        return TurretAngle.fromMotorRotations(turretMotor.getEncoder().getPosition());
    }

    public Optional<TurretAngle> getTargetAngle() {
        return targetAngle;
    }

    public void setEncoderPosition(TurretAngle angle) {
        turretMotor.getEncoder().setPosition(angle.asMotorRotations());
    }

    public void dutyCycle(double duty) {
        turretMotor.set(duty);
        targetAngle = Optional.empty();
    }

    public void setIsCalibrated(boolean calibrated) {
        isCalibrated = calibrated;
    }

    public boolean getIsCalibrated() {
        return isCalibrated;
    }

    public void logTargetPoint(Optional<Translation2d> point) {
        if (targetPointPublisher.isEmpty()) return;
        targetPointPublisher.get().set(point == null ? null : point.orElse(null));
    }

    public Pose2d getTurretPoseOnField() {
        Pose2d robotPose = RobotContainer.getInstance().swerve.getState().Pose;
        Translation2d turretCenter =
                robotPose.getTranslation().plus(TurretConstants.turretCenterOffset.rotateBy(robotPose.getRotation()));
        Pose2d turretPose = new Pose2d(
                turretCenter, new Rotation2d(getCurretAngle().asMechanismAngle()).plus(robotPose.getRotation()));
        return turretPose;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(TurretConstants.subsystemName, this);

        Logger.logSparkMotor(TurretConstants.subsystemName, "motor", turretMotor);

        Logger.logDouble(
                TurretConstants.subsystemName,
                "currentTurretDeg",
                getCurretAngle().asMechanismAngle().in(Degrees));

        Logger.logDouble(
                TurretConstants.subsystemName,
                "targetTurretDeg",
                targetAngle.map((val) -> val.asMotorAngle().in(Degrees)).orElse(Double.NaN));
        Logger.logDouble(
                TurretConstants.subsystemName,
                "targetMotorRots",
                targetAngle.map((val) -> val.asMechanismRotations()).orElse(Double.NaN));
        Logger.logBool(TurretConstants.subsystemName, "calibrationLimit", isAtCalibrationLimit());
        Logger.logBool(TurretConstants.subsystemName, "isCalibrated", getIsCalibrated());

        Pose2d robotPose = RobotContainer.getInstance().swerve.getState().Pose;
        Translation2d turretCenter =
                robotPose.getTranslation().plus(TurretConstants.turretCenterOffset.rotateBy(robotPose.getRotation()));

        if (turretPosePublisher.isPresent()) {
            turretPosePublisher.get().set(getTurretPoseOnField());
        }
        if (targetTurretPosePublisher.isPresent()) {
            Optional<Pose2d> targetTurretPose;
            if (targetAngle.isPresent()) {
                targetTurretPose = Optional.of(new Pose2d(
                        turretCenter,
                        new Rotation2d(targetAngle.get().asMechanismAngle()).plus(robotPose.getRotation())));
            } else {
                targetTurretPose = Optional.empty();
            }

            targetTurretPosePublisher.get().set(targetTurretPose.orElse(null));
        }

        if (getCurrentCommand() == null) {
            logTargetPoint(Optional.empty());
        }
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean turretConnected = SparkUtils.isConnected(turretMotor);

        CANMonitor.logCANDeviceStatus("turretMotor", TurretConstants.motorID, turretConnected);
        Alerts.turretDisconnected.set(!turretConnected);
        Alerts.turretOverheating.set(turretMotor.getMotorTemperature() >= 80);
        Alerts.turretFaults.set(SparkUtils.hasCriticalFaults(turretMotor.getFaults()));
        Alerts.turretWarnings.set(SparkUtils.hasCriticalWarnings(turretMotor.getWarnings()));
    }
}
