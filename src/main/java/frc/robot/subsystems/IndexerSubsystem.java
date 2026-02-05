package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.units.SpindexerVelocity;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax spindexerMotor; // MAKES MOTOR
    private final SparkMax gateMotor;

    private Optional<SpindexerVelocity> spindexerTarget;

    public IndexerSubsystem() {
        spindexerMotor = new SparkMax(
                IndexerConstants.Spindexer.motorID, MotorType.kBrushless); // port number under IndexerConstants
        SparkMaxConfig spindexerConfig = new SparkMaxConfig();
        SparkUtils.configureBasicSettings(
                spindexerConfig,
                IndexerConstants.Spindexer.maxCurrent,
                IndexerConstants.Spindexer.idleMode,
                IndexerConstants.Spindexer.inverted,
                IndexerConstants.Spindexer.maxDutyCycle,
                IndexerConstants.Spindexer.nominalVoltage);
        SparkUtils.configureClosedLoopSettings(
                spindexerConfig,
                IndexerConstants.Spindexer.kP,
                IndexerConstants.Spindexer.kI,
                IndexerConstants.Spindexer.kD,
                IndexerConstants.Spindexer.kStaticG,
                IndexerConstants.Spindexer.kCos,
                IndexerConstants.Spindexer.kS,
                IndexerConstants.Spindexer.kV,
                IndexerConstants.Spindexer.kA);
        if (spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure spindexer motor");
            Alerts.spindexerConfigFail.set(true);
        }

        gateMotor = new SparkMax(
                IndexerConstants.Gate.motorID,
                MotorType.kBrushless); // port number in IndexerConstants; defines the motor as brushless
        SparkMaxConfig gateConfig = new SparkMaxConfig();
        SparkUtils.configureBasicSettings(
                gateConfig,
                IndexerConstants.Gate.maxCurrent,
                IndexerConstants.Gate.idleMode,
                IndexerConstants.Gate.inverted,
                IndexerConstants.Gate.maxDutyCycle,
                IndexerConstants.Gate.nominalVoltage);
        if (gateMotor.configure(gateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure gate motor");
            Alerts.gateConfigFail.set(true);
        }

        spindexerTarget = Optional.empty();
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(IndexerConstants.subsystemName, this);

        Logger.logSparkMotor(IndexerConstants.subsystemName, "spindexerMotor", spindexerMotor);
        Alerts.spindexerDisconnected.set(!SparkUtils.isConnected(spindexerMotor));
        Alerts.spindexerOverheating.set(spindexerMotor.getMotorTemperature() >= 80);
        Alerts.spindexerFaults.set(SparkUtils.hasCriticalFaults(spindexerMotor.getFaults()));
        Alerts.spindexerWarnings.set(SparkUtils.hasCriticalWarnings(spindexerMotor.getWarnings()));

        Logger.logSparkMotor(IndexerConstants.subsystemName, "gateMotor", gateMotor);
        Alerts.gateDisconnected.set(!SparkUtils.isConnected(gateMotor));
        Alerts.gateOverheating.set(gateMotor.getMotorTemperature() >= 80);
        Alerts.gateFaults.set(SparkUtils.hasCriticalFaults(gateMotor.getFaults()));
        Alerts.gateWarnings.set(SparkUtils.hasCriticalWarnings(gateMotor.getWarnings()));

        Logger.logDouble(
                IndexerConstants.subsystemName,
                "spindexerMotor/targetMechanismRPM",
                spindexerTarget.map((val) -> val.getAsMechanismRPM()).orElse(Double.NaN));
        Logger.logBool(
                IndexerConstants.subsystemName,
                "spindexerMotor/isAtTarget",
                isSpindexerAtTarget().orElse(true));
    }

    public SpindexerVelocity getSpindexerVelocity() {
        return SpindexerVelocity.fromMotorRPM(
                spindexerMotor.getEncoder().getVelocity()); // to check velocity for spinup
    }

    public void setSpindexerMotorVelocity(SpindexerVelocity vel) {
        if (vel == null) {
            Logger.reportWarning("Cannot set spindexer velocity to a null velocity", true);
            return;
        }

        spindexerTarget = Optional.of(vel);

        double request = vel.getAsMotorRPM();
        spindexerMotor
                .getClosedLoopController()
                .setSetpoint(request, ControlType.kVelocity); // says that velocity controls velocity
    }

    public void stopSpindexer() {
        spindexerTarget = Optional.empty();

        spindexerMotor.stopMotor();
    }

    public void setGateMotorVoltage(double voltage) {
        gateMotor.setVoltage(voltage);
    }

    public void stopAll() {
        spindexerMotor.stopMotor(); // safety
        gateMotor.stopMotor();
    }

    public Optional<Boolean> isSpindexerAtTarget() {
        if (spindexerTarget.isEmpty()) return Optional.empty();

        double motorRPM = getSpindexerVelocity().getAsMechanismRPM();
        double targetRPM = spindexerTarget.get().getAsMechanismRPM();
        double toleranceRPM = IndexerConstants.Spindexer.targetTolerance.getAsMechanismRPM();

        return Optional.of(Math.abs(targetRPM - motorRPM) <= toleranceRPM);
    }
}
