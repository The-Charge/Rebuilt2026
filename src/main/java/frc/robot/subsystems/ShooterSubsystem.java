package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.units.ShooterVelocity;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shooterMotor;

    private Optional<ShooterVelocity> shooterTarget;

    public ShooterSubsystem() {
        shooterMotor =
                new SparkFlex(ShooterConstants.motorID, MotorType.kBrushless); // port number under IndexerConstants
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        SparkUtils.configureBasicSettings(
                shooterConfig,
                ShooterConstants.maxCurrent,
                ShooterConstants.idleMode,
                ShooterConstants.inverted,
                ShooterConstants.maxDutyCycle,
                ShooterConstants.nominalVoltage);
        SparkUtils.configureClosedLoopSettings(
                shooterConfig,
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kStaticG,
                ShooterConstants.kCos,
                ShooterConstants.kS,
                ShooterConstants.kV,
                ShooterConstants.kA);
        if (shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure shooter motor");
            Alerts.shooterConfigFail.set(true);
        }

        shooterTarget = Optional.empty();
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(ShooterConstants.subsystemName, this);

        Logger.logSparkMotor(ShooterConstants.subsystemName, "shooterMotor", shooterMotor);
        CANMonitor.logCANDeviceStatus("shooterMotor", ShooterConstants.motorID, SparkUtils.isConnected(shooterMotor));
        Alerts.shooterDisconnected.set(!SparkUtils.isConnected(shooterMotor));
        Alerts.shooterOverheating.set(shooterMotor.getMotorTemperature() >= 80);
        Alerts.shooterFaults.set(SparkUtils.hasCriticalFaults(shooterMotor.getFaults()));
        Alerts.shooterWarnings.set(SparkUtils.hasCriticalWarnings(shooterMotor.getWarnings()));

        Logger.logDouble(
                ShooterConstants.subsystemName,
                "shooter/targetMechanismRPM",
                shooterTarget.map((val) -> val.getAsMechanismRPM()).orElse(Double.NaN));
        Logger.logBool(
                ShooterConstants.subsystemName,
                "shooterMotor/isAtTarget",
                isShooterAtTarget().orElse(true));
    }

    public ShooterVelocity getShooterVelocity() { // change to shooter
        return ShooterVelocity.fromMotorRPM(shooterMotor.getEncoder().getVelocity()); // to check velocity for spinup
    }

    public void setShooterMotorVelocity(ShooterVelocity vel) { // change to shooter
        if (vel == null) {
            Logger.reportWarning("Cannot set shooter velocity to a null velocity", true);
            return;
        }

        shooterTarget = Optional.of(vel);

        double request = vel.getAsMotorRPM();
        shooterMotor
                .getClosedLoopController()
                .setSetpoint(request, ControlType.kVelocity); // says that velocity controls velocity
    }

    public void stopShooter() {
        shooterTarget = Optional.empty();

        shooterMotor.stopMotor();
    }

    public void stopAll() {
        shooterMotor.stopMotor();
    }

    public Optional<Boolean> isShooterAtTarget() {
        if (shooterTarget.isEmpty()) return Optional.empty();

        double motorRPM = getShooterVelocity().getAsMechanismRPM();
        double targetRPM = shooterTarget.get().getAsMechanismRPM();
        double toleranceRPM = ShooterConstants.targetTolerance.getAsMechanismRPM();

        return Optional.of(Math.abs(targetRPM - motorRPM) <= toleranceRPM);
    }
}
