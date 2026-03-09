package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax spindexerMotor;
    private final SparkMax exchangeMotor;

    public IndexerSubsystem() {
        spindexerMotor = new SparkMax(IndexerConstants.Spindexer.motorID, MotorType.kBrushless);
        SparkMaxConfig spindexerConfig = new SparkMaxConfig();
        SparkUtils.configureBasicSettings(
                spindexerConfig,
                IndexerConstants.Spindexer.maxCurrent,
                IndexerConstants.Spindexer.idleMode,
                IndexerConstants.Spindexer.inverted,
                IndexerConstants.Spindexer.maxDutyCycle,
                IndexerConstants.Spindexer.nominalVoltage);
        if (spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure spindexer motor");
            Alerts.spindexerConfigFail.set(true);
        }

        exchangeMotor = new SparkMax(
                IndexerConstants.Exchange.motorID,
                MotorType.kBrushless); // port number in IndexerConstants; defines the motor as brushless
        SparkMaxConfig gateConfig = new SparkMaxConfig();
        SparkUtils.configureBasicSettings(
                gateConfig,
                IndexerConstants.Exchange.maxCurrent,
                IndexerConstants.Exchange.idleMode,
                IndexerConstants.Exchange.inverted,
                IndexerConstants.Exchange.maxDutyCycle,
                IndexerConstants.Exchange.nominalVoltage);
        if (exchangeMotor.configure(gateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure exchange motor");
            Alerts.exchangeConfigFail.set(true);
        }
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(IndexerConstants.subsystemName, this);

        Logger.logSparkMotor(IndexerConstants.subsystemName, "spindexerMotor", spindexerMotor);

        Logger.logSparkMotor(IndexerConstants.subsystemName, "exchangeMotor", exchangeMotor);
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean spindexerConnected = SparkUtils.isConnected(spindexerMotor);

        CANMonitor.logCANDeviceStatus("spindexerMotor", IndexerConstants.Spindexer.motorID, spindexerConnected);
        Alerts.spindexerDisconnected.set(!spindexerConnected);
        Alerts.spindexerOverheating.set(spindexerMotor.getMotorTemperature() >= 80);
        Alerts.spindexerFaults.set(SparkUtils.hasCriticalFaults(spindexerMotor.getFaults()));
        Alerts.spindexerWarnings.set(SparkUtils.hasCriticalWarnings(spindexerMotor.getWarnings()));

        boolean exchangeConnected = SparkUtils.isConnected(exchangeMotor);
        CANMonitor.logCANDeviceStatus("exchangeMotor", IndexerConstants.Exchange.motorID, exchangeConnected);
        Alerts.exchangeDisconnected.set(!exchangeConnected);
        Alerts.exchangeOverheating.set(exchangeMotor.getMotorTemperature() >= 80);
        Alerts.exchangeFaults.set(SparkUtils.hasCriticalFaults(exchangeMotor.getFaults()));
        Alerts.exchangeWarnings.set(SparkUtils.hasCriticalWarnings(exchangeMotor.getWarnings()));
    }

    public double getSpindexerVoltage() {
        return spindexerMotor.get();
    }

    public void setSpindexerVoltage(double volts) {
        spindexerMotor.setVoltage(volts);
    }

    public void stopSpindexer() {
        spindexerMotor.stopMotor();
    }

    public void setExchangeMotorVoltage(double voltage) {
        exchangeMotor.setVoltage(voltage);
    }

    public void stopAll() {
        spindexerMotor.stopMotor(); // safety
        exchangeMotor.stopMotor();
    }
}
