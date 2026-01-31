package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.units.SpindexerVelocity;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import frc.robot.utils.TalonFXUtils;
import java.util.Optional;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX spindexerMotor; // MAKES MOTOR
    private final SparkMax gateMotor;

    private Optional<SpindexerVelocity> spindexerTarget;

    public IndexerSubsystem() {
        spindexerMotor = new TalonFX(IndexerConstants.Spindexer.motorID); // port number under IndexerConstants
        TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
        TalonFXUtils.configureBasicSettings(
                spindexerConfig,
                IndexerConstants.Spindexer.maxCurrent,
                IndexerConstants.Spindexer.neutralMode,
                IndexerConstants.Spindexer.inverted,
                IndexerConstants.Spindexer.maxDutyCycle,
                Optional.of(IndexerConstants.Spindexer.maxVoltage));
        TalonFXUtils.configureClosedLoopSettings(
                spindexerConfig,
                IndexerConstants.Spindexer.kP,
                IndexerConstants.Spindexer.kI,
                IndexerConstants.Spindexer.kD,
                Optional.empty(),
                Optional.empty());
        if (spindexerMotor.getConfigurator().apply(spindexerConfig) != StatusCode.OK) {
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

        Logger.logTalonFX(IndexerConstants.subsystemName, "spindexerMotor", spindexerMotor);
        Alerts.spindexerDisconnected.set(!spindexerMotor.isConnected());
        Alerts.spindexerOverheating.set(
                spindexerMotor.getDeviceTemp().getValue().abs(Units.Celsius) >= 80);
        Alerts.spindexerFaults.set(
                TalonFXUtils.getAllActiveFaults(spindexerMotor).hasCriticalFaults());

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
        return SpindexerVelocity.fromMechanismVelocity(
                spindexerMotor.getVelocity().getValue()); // to check velocity for spinup
    }

    public void setSpindexerMotorVelocity(SpindexerVelocity vel) {
        if (vel == null) {
            Logger.reportWarning("Cannot set spindexer velocity to a null velocity", true);
            return;
        }

        spindexerTarget = Optional.of(vel);

        VelocityVoltage request = new VelocityVoltage(vel.getAsMechanismVelocity());
        spindexerMotor.setControl(request); // says that velocity controls velocity
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
