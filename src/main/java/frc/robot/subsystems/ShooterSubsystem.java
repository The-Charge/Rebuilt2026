package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.Motor;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shootMotor;
    private final SysIdRoutine sysIdRoutine;

    private final Alert motorDisconnected, motorOverheating, motorFaults, motorWarnings, motorConfigFail;

    private Optional<AngularVelocity> targetShooterSpeed;
    private SequentialCommandGroup runSysId;

    public ShooterSubsystem() {
        motorDisconnected = Alerts.makeDisconnectAlert(Motor.motorName, Motor.motorID);
        motorOverheating = Alerts.makeOverheatingAlert(Motor.motorName, Motor.motorID);
        motorFaults = Alerts.makeCriticalFaultsAlert(Motor.motorName, Motor.motorID);
        motorWarnings = Alerts.makeCriticalWarningsAlert(Motor.motorName, Motor.motorID);
        motorConfigFail = Alerts.makeConfigFailAlert(Motor.motorName, Motor.motorID);

        shootMotor = new SparkFlex(Motor.motorID, MotorType.kBrushless);
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
    public String getName() {
        return ShooterConstants.subsystemName;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(getName(), this);

        Logger.logSparkMotor(getName(), Motor.motorName, shootMotor);

        Logger.logDouble(
                getName(),
                "targetMotorRPM",
                targetShooterSpeed.map((val) -> val.in(RPM)).orElse(Double.NaN));
        Logger.logBool(getName(), "isAtTarget", isShooterAtTargetSpeed().orElse(true));
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean shootConnected = SparkUtils.isConnected(shootMotor);

        CANMonitor.logCANDeviceStatus(Motor.motorName, Motor.motorID, shootConnected);
        motorDisconnected.set(!shootConnected);
        motorOverheating.set(shootMotor.getMotorTemperature() >= 80);
        motorWarnings.set(SparkUtils.hasCriticalWarnings(shootMotor.getWarnings()));
        motorFaults.set(SparkUtils.hasCriticalFaults(shootMotor.getFaults()));
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
                Motor.currentLimit,
                Motor.idleMode,
                Motor.inverted,
                Motor.maxDutyCycle,
                Motor.nominalVoltage);
        SparkUtils.configureClosedLoopSettings(
                shootConfig,
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

        if (!SparkUtils.safeApplyConfig(shootMotor, Motor.motorName, shootConfig)) {
            motorConfigFail.set(true);
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
