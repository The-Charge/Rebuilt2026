package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.TurretConstants.Motor;
import frc.robot.units.TurretAngle;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax turretMotor;

    private final Alert motorDisconnected, motorOverheating, motorFaults, motorWarnings, motorConfigFail;

    private Optional<TurretAngle> targetAngle;
    private boolean isCalibrated;

    private final Optional<StructPublisher<Pose2d>> turretPosePublisher;
    private final Optional<StructPublisher<Pose2d>> targetTurretPosePublisher;
    private final Optional<StructPublisher<Translation2d>> targetPointPublisher;
    private final Optional<StructPublisher<Translation2d>> targetPredictedPointPublisher;
    private final Optional<StructPublisher<Translation2d>> predictedOffsetPublisher;

    public TurretSubsystem() {
        motorDisconnected = Alerts.makeDisconnectAlert(Motor.motorName, Motor.motorID);
        motorOverheating = Alerts.makeOverheatingAlert(Motor.motorName, Motor.motorID);
        motorFaults = Alerts.makeCriticalFaultsAlert(Motor.motorName, Motor.motorID);
        motorWarnings = Alerts.makeCriticalWarningsAlert(Motor.motorName, Motor.motorID);
        motorConfigFail = Alerts.makeConfigFailAlert(Motor.motorName, Motor.motorID);

        turretMotor = new SparkMax(Motor.motorID, MotorType.kBrushless);

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        SparkUtils.configureBasicSettings(
                turretConfig,
                Motor.maxCurrent,
                Motor.idleMode,
                Motor.inverted,
                Motor.maxDutyCycle,
                Motor.nominalVoltage);
        SparkUtils.configureClosedLoopSettings(
                turretConfig,
                Motor.kP,
                Motor.kI,
                Motor.iZone,
                Motor.kD,
                Motor.kStaticG,
                Motor.kCos,
                Motor.kS,
                Motor.kV,
                Motor.kA,
                Motor.rampTime);
        SparkUtils.configureLimitSwitches(
                turretConfig,
                Motor.forwardHardLimitEnabled,
                Motor.forwardHardLimitResetRots,
                Motor.reverseHardLimitEnabled,
                Motor.reverseHardLimitResetRots);
        SparkUtils.configureMAXMotion(turretConfig, Motor.maxAccel, Motor.cruiseVel, Motor.allowedError);

        if (!SparkUtils.safeApplyConfig(turretMotor, Motor.motorName, turretConfig)) {
            motorConfigFail.set(true);
        }

        targetAngle = Optional.empty();
        isCalibrated = false;

        turretPosePublisher = Logger.makeStructPublisher(getName(), "turretPose", Pose2d.struct);
        targetTurretPosePublisher = Logger.makeStructPublisher(getName(), "targetTurretPose", Pose2d.struct);
        targetPointPublisher = Logger.makeStructPublisher(getName(), "targetPoint", Translation2d.struct);
        targetPredictedPointPublisher =
                Logger.makeStructPublisher(getName(), "targetPredictedPoint", Translation2d.struct);
        predictedOffsetPublisher = Logger.makeStructPublisher(getName(), "predictedOffset", Translation2d.struct);
    }

    @Override
    public String getName() {
        return TurretConstants.subsystemName;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(getName(), this);

        Logger.logSparkMotor(getName(), Motor.motorName, turretMotor);

        Logger.logDouble(
                getName(),
                "currentTurretDeg",
                getCurretAngle().asMechanismAngle().in(Degrees));

        Logger.logDouble(
                getName(),
                "targetTurretDeg",
                targetAngle.map((val) -> val.asMechanismAngle().in(Degrees)).orElse(Double.NaN));
        Logger.logDouble(
                getName(),
                "targetMotorRots",
                targetAngle.map((val) -> val.asMotorRotations()).orElse(Double.NaN));
        Logger.logBool(getName(), "isAtCalibrationLimit", isAtCalibrationLimit());
        Logger.logBool(getName(), "isCalibrated", getIsCalibrated());

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

            targetTurretPosePublisher
                    .get()
                    .set(targetTurretPose.orElse(new Pose2d(Double.NaN, Double.NaN, new Rotation2d(Double.NaN))));
        }

        if (getCurrentCommand() == null) {
            logTargetPoint(Optional.empty());
            logTargetPredictedPoint(Optional.empty());
            logPredictedOffset(Optional.empty());
        }
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean turretConnected = SparkUtils.isConnected(turretMotor);

        CANMonitor.logCANDeviceStatus(Motor.motorName, Motor.motorID, turretConnected);
        motorDisconnected.set(!turretConnected);
        motorOverheating.set(turretMotor.getMotorTemperature() >= 80);
        motorFaults.set(SparkUtils.hasCriticalFaults(turretMotor.getFaults()));
        motorWarnings.set(SparkUtils.hasCriticalWarnings(turretMotor.getWarnings()));
    }

    public void setTurretAngle(TurretAngle angle) {
        if (angle == null) {
            Logger.reportWarning(String.format("Cannot rotate %s to a null angle", Motor.motorName), true);
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
        return turretMotor.getOutputCurrent() > Motor.calibrationThresholdCurrent.in(Amps)
                || Math.abs(turretMotor.getEncoder().getVelocity()) < Motor.calibrationThresholdVel.in(RPM);
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

    public Optional<Boolean> isAtTarget() {
        if (targetAngle.isEmpty()) return Optional.empty();

        double tolerance = TurretConstants.targetTolerance.asMotorRotations();
        double target = targetAngle.get().asMotorRotations();
        double current = getCurretAngle().asMotorRotations();

        return Optional.of(Math.abs(target - current) <= tolerance);
    }

    public void logTargetPoint(Optional<Translation2d> point) {
        if (targetPointPublisher.isEmpty()) return;
        targetPointPublisher.get().set(point.orElse(new Translation2d(Double.NaN, Double.NaN)));
    }

    public void logTargetPredictedPoint(Optional<Translation2d> point) {
        if (targetPredictedPointPublisher.isEmpty()) return;
        targetPredictedPointPublisher.get().set(point.orElse(new Translation2d(Double.NaN, Double.NaN)));
    }

    public void logPredictedOffset(Optional<Translation2d> offset) {
        if (predictedOffsetPublisher.isEmpty()) return;
        predictedOffsetPublisher.get().set(offset.orElse(new Translation2d(Double.NaN, Double.NaN)));
    }

    public Pose2d getTurretPoseOnField() {
        Pose2d robotPose = RobotContainer.getInstance().swerve.getState().Pose;
        Translation2d turretCenter =
                robotPose.getTranslation().plus(TurretConstants.turretCenterOffset.rotateBy(robotPose.getRotation()));
        Pose2d turretPose = new Pose2d(
                turretCenter, new Rotation2d(getCurretAngle().asMechanismAngle()).plus(robotPose.getRotation()));
        return turretPose;
    }
}
