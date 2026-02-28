package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.ShootConfig;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shootMotor;
    // private final Servo hood; // this will likely be a motor, not a servo
    // private final SparkMax hoodMotor;

    // public enum HoodPos {
    //     UP,
    //     DOWN,
    // }

    // private HoodPos hoodPos;
    private Optional<AngularVelocity> targetShooterSpeed;

    public ShooterSubsystem() {
        shootMotor = new SparkFlex(ShootConfig.motorID, MotorType.kBrushless);
        // hoodMotor = new SparkMax(HoodConfig.ID, MotorType.kBrushless);

        // hoodPos = HoodPos.DOWN;
        targetShooterSpeed = Optional.empty();

        configureMotor();
    }

    // public void setHoodPos(HoodPos hp) {
    //     hoodPos = hp;

    //     Rotation2d angle = hoodPos == HoodPos.UP ? ShooterConstants.upAngle : ShooterConstants.downAngle;

    //     hoodMotor
    //             .getClosedLoopController()
    //             .setSetpoint(angle.getRadians() * HoodConfig.ticksPerRadian, ControlType.kPosition);
    // }

    public void shoot(AngularVelocity speed) {
        shootMotor.getClosedLoopController().setSetpoint(speed.abs(RPM), ControlType.kVelocity);
        targetShooterSpeed = Optional.of(speed);
    }

    public void stopShoot() {
        shootMotor.stopMotor();
        targetShooterSpeed = Optional.empty();
    }

    // public HoodPos getHoodPos() {
    //     return hoodPos;
    // }

    private void configureMotor() {
        // shoot
        SparkFlexConfig shootConfig = new SparkFlexConfig();

        SparkUtils.configureBasicSettings(
                shootConfig,
                ShootConfig.currentLimit,
                ShootConfig.idleMode,
                ShootConfig.inverted,
                ShootConfig.maxDutyCycle,
                ShootConfig.nominalVoltage);
        SparkUtils.configureClosedLoopSettings(
                shootConfig,
                ShootConfig.kP,
                ShootConfig.kI,
                ShootConfig.kD,
                ShootConfig.kStaticG,
                ShootConfig.kCos,
                ShootConfig.kS,
                ShootConfig.kV,
                ShootConfig.kA,
                ShootConfig.iZone);

        if (shootMotor.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure shooter motor");
            Alerts.shooterConfigFail.set(true);
        }

        // hood
        // I'm not going to configure this properly yet as I don't know if we will even have a motor for this
        // SparkMaxConfig hoodConfig = new SparkMaxConfig();

        // hoodConfig.closedLoop.pid(HoodConfig.p, HoodConfig.i, HoodConfig.d);

        // hoodConfig.idleMode(HoodConfig.idleMode);
        // hoodConfig.smartCurrentLimit(HoodConfig.currentLimit);
        // hoodConfig.inverted(HoodConfig.inverted);

        // hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Optional<Boolean> isShooterAtTargetSpeed() {
        if (targetShooterSpeed.isEmpty()) return Optional.empty();

        double currentRPM = shootMotor.getEncoder().getVelocity();
        double targetRPM = targetShooterSpeed.get().in(RPM);
        double toleranceRPM = ShooterConstants.ShootConfig.targetTolerance.in(RPM);

        return Optional.of(Math.abs(currentRPM - targetRPM) <= toleranceRPM);
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(ShooterConstants.subsystemName, this);

        Logger.logSparkMotor(ShooterConstants.subsystemName, "shoot", shootMotor);
        CANMonitor.logCANDeviceStatus(
                "shootMotor", ShooterConstants.ShootConfig.motorID, SparkUtils.isConnected(shootMotor));
        Alerts.shooterDisconnected.set(!SparkUtils.isConnected(shootMotor));
        Alerts.shooterOverheating.set(shootMotor.getMotorTemperature() >= 80);
        Alerts.shooterWarnings.set(SparkUtils.hasCriticalWarnings(shootMotor.getWarnings()));
        Alerts.shooterFaults.set(SparkUtils.hasCriticalFaults(shootMotor.getFaults()));

        // Logger.logSparkMotor(ShooterConstants.subsystemName, "hood", hoodMotor);

        // Alerts.hoodDisconnected.set(!SparkUtils.isConnected(hoodMotor));
        // Alerts.hoodOverheating.set(hoodMotor.getMotorTemperature() >= 80);
        // Alerts.hoodWarnings.set(SparkUtils.hasCriticalWarnings(hoodMotor.getWarnings()));
        // Alerts.hoodFaults.set(SparkUtils.hasCriticalFaults(hoodMotor.getFaults()));

        // Logger.logString(ShooterConstants.subsystemName, "hood/angle state", hoodPos == HoodPos.UP ? "Up" : "Down");

        Logger.logDouble(
                ShooterConstants.subsystemName,
                "shoot/targetMotorRPM",
                targetShooterSpeed.map((val) -> val.in(RPM)).orElse(Double.NaN));
        Logger.logBool(
                ShooterConstants.subsystemName,
                "shoot/isAtTarget",
                isShooterAtTargetSpeed().orElse(true));
    }
}
