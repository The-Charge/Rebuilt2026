package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;
import frc.robot.units.TurretAngle;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

// shoot angle, turret
public class TurretSubsystem extends SubsystemBase {
    // private final SparkFlex shooter;
    // private final Servo hood;

    private final SparkMax turretMotor;
    private final DigitalInput forwardLimit;

    private Optional<TurretAngle> targetAngle;
    private boolean isCalibrated;

    public TurretSubsystem() {
        forwardLimit = new DigitalInput(TurretConstants.forwardLimitChannel);

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

        // SmartDashboard.putNumber("gearRatio", 27);

        // shooter = new SparkFlex(TurretConstants.shooterId, MotorType.kBrushless);
        // hood = new Servo(TurretConstants.hoodChannel);
    }

    public void setTurretAngle(TurretAngle angle) {
        if (angle == null) {
            Logger.reportWarning("Cannot set turret angle to a null angle", true);
            return;
        }
        angle = angle.wrap();
        if (!angle.isLegal()) return;

        targetAngle = Optional.of(angle);

        Logger.logDouble(TurretConstants.subsystemName, "limitedMotorRots", angle.asMotorRotations());

        turretMotor.getClosedLoopController().setSetpoint(angle.asMotorRotations(), ControlType.kPosition);
    }

    public void stop() {
        turretMotor.set(0);
        targetAngle = Optional.empty();
    }

    // Checks if turret is at hard stop by checking if output current is greater than threshold
    public boolean isAtReverseLimit() {
        return turretMotor.getOutputCurrent() > TurretConstants.calibrationThresholdCurrent
                || Math.abs(turretMotor.getEncoder().getVelocity()) < 1;
    }

    public TurretAngle getTurretAngle() {
        return TurretAngle.fromMotorRotations(turretMotor.getEncoder().getPosition());
    }

    public Optional<TurretAngle> getTargetAngle() {
        return targetAngle;
    }

    public void setEncoderPosition(TurretAngle angle) {
        turretMotor.getEncoder().setPosition(angle.asMotorRotations());
    }
    /**
     * sets percent output of motor
     **/
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

    @Override
    public void periodic() {
        Logger.logSubsystem(TurretConstants.subsystemName, this);

        Logger.logSparkMotor(TurretConstants.subsystemName, "motor", turretMotor);

        Logger.logDouble(
                TurretConstants.subsystemName,
                "currentTurretDeg",
                getTurretAngle().asMechanismAngle().in(Degrees));

        Logger.logDouble(
                TurretConstants.subsystemName,
                "targetTurretDeg",
                targetAngle.map((val) -> val.asMotorAngle().in(Degrees)).orElse(Double.NaN));
        Logger.logDouble(
                TurretConstants.subsystemName,
                "targetMotorRots",
                targetAngle.map((val) -> val.asMechanismRotations()).orElse(Double.NaN));
        Logger.logBool(TurretConstants.subsystemName, "forwardLimit", isAtReverseLimit());
        Logger.logBool(TurretConstants.subsystemName, "isCalibrated", getIsCalibrated());
        Logger.logBool(
                TurretConstants.subsystemName,
                "isTargetLegal",
                targetAngle.map((val) -> val.isLegal()).orElse(true));
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
