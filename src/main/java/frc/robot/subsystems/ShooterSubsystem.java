package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shootMotor;
    private final SysIdRoutine sysIdRoutine;

    private Optional<AngularVelocity> targetShooterSpeed;
    private SequentialCommandGroup runSysId;

    public ShooterSubsystem() {
        shootMotor = new SparkFlex(ShooterConstants.motorID, MotorType.kBrushless);
        configureMotor();

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::setVoltage, this::logSysIdMotors, this));
        runSysId = new SequentialCommandGroup(
                sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                sysIdDynamic(SysIdRoutine.Direction.kForward),
                sysIdDynamic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("sysIdShooter", runSysId);

        targetShooterSpeed = Optional.empty();
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(ShooterConstants.subsystemName, this);

        Logger.logSparkMotor(ShooterConstants.subsystemName, "shoot", shootMotor);

        Logger.logDouble(
                ShooterConstants.subsystemName,
                "targetMotorRPM",
                targetShooterSpeed.map((val) -> val.in(RPM)).orElse(Double.NaN));
        Logger.logBool(
                ShooterConstants.subsystemName,
                "isAtTarget",
                isShooterAtTargetSpeed().orElse(true));
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean shootConnected = SparkUtils.isConnected(shootMotor);

        CANMonitor.logCANDeviceStatus("shootMotor", ShooterConstants.motorID, shootConnected);
        Alerts.shooterDisconnected.set(!shootConnected);
        Alerts.shooterOverheating.set(shootMotor.getMotorTemperature() >= 80);
        Alerts.shooterWarnings.set(SparkUtils.hasCriticalWarnings(shootMotor.getWarnings()));
        Alerts.shooterFaults.set(SparkUtils.hasCriticalFaults(shootMotor.getFaults()));
    }

    private void logSysIdMotors(SysIdRoutineLog log) {
        MotorLog motorLog = log.motor("shooter");
        motorLog.angularPosition(Rotations.of(shootMotor.getEncoder().getPosition()));
        motorLog.angularVelocity(RPM.of(shootMotor.getEncoder().getVelocity()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void setVoltage(Voltage voltage) {
        shootMotor.setVoltage(voltage.in(Volts));
        targetShooterSpeed = Optional.empty();
    }

    public void setTargetVelocity(AngularVelocity speed) {
        shootMotor.getClosedLoopController().setSetpoint(speed.abs(RPM), ControlType.kVelocity);
        targetShooterSpeed = Optional.of(speed);
    }

    public void stopShooter() {
        shootMotor.stopMotor();
        targetShooterSpeed = Optional.empty();
    }

    private void configureMotor() {
        SparkFlexConfig shootConfig = new SparkFlexConfig();

        SparkUtils.configureBasicSettings(
                shootConfig,
                ShooterConstants.currentLimit,
                ShooterConstants.idleMode,
                ShooterConstants.inverted,
                ShooterConstants.maxDutyCycle,
                ShooterConstants.nominalVoltage);
        SparkUtils.configureClosedLoopSettings(
                shootConfig,
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD,
                ShooterConstants.kStaticG,
                ShooterConstants.kCos,
                ShooterConstants.kS,
                ShooterConstants.kV,
                ShooterConstants.kA,
                ShooterConstants.iZone);

        if (shootMotor.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure shooter motor");
            Alerts.shooterConfigFail.set(true);
        }
    }

    public Optional<Boolean> isShooterAtTargetSpeed() {
        if (targetShooterSpeed.isEmpty()) return Optional.empty();

        double currentRPM = shootMotor.getEncoder().getVelocity();
        double targetRPM = targetShooterSpeed.get().in(RPM);
        double toleranceRPM = ShooterConstants.targetTolerance.in(RPM);

        return Optional.of(Math.abs(currentRPM - targetRPM) <= toleranceRPM);
    }
}
