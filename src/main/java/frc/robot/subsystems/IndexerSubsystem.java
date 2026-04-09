package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IndexerConstants.Exchange;
import frc.robot.constants.IndexerConstants.Spindexer;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import frc.robot.utils.TalonFXUtils;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX spindexerMotor;
    private final SparkMax exchangeMotor;

    private final Alert spindexerDisconnected, spindexerOverheating, spindexerFaults, spindexerConfigFail;
    private final Alert exchangeConfigFail, exchangeDisconnected, exchangeOverheating, exchangeFaults, exchangeWarnings;

    public IndexerSubsystem() {
        spindexerDisconnected = Alerts.makeDisconnectAlert(Spindexer.motorName, Spindexer.motorID);
        spindexerOverheating = Alerts.makeOverheatingAlert(Spindexer.motorName, Spindexer.motorID);
        spindexerFaults = Alerts.makeCriticalFaultsAlert(Spindexer.motorName, Spindexer.motorID);
        spindexerConfigFail = Alerts.makeConfigFailAlert(Spindexer.motorName, Spindexer.motorID);

        exchangeConfigFail = Alerts.makeConfigFailAlert(Exchange.motorName, Exchange.motorID);
        exchangeDisconnected = Alerts.makeDisconnectAlert(Exchange.motorName, Exchange.motorID);
        exchangeOverheating = Alerts.makeOverheatingAlert(Exchange.motorName, Exchange.motorID);
        exchangeFaults = Alerts.makeCriticalFaultsAlert(Exchange.motorName, Exchange.motorID);
        exchangeWarnings = Alerts.makeCriticalWarningsAlert(Exchange.motorName, Exchange.motorID);

        spindexerMotor = new TalonFX(Spindexer.motorID);
        TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
        TalonFXUtils.configureBasicSettings(
                spindexerConfig,
                Spindexer.maxCurrent,
                Spindexer.neutralMode,
                Spindexer.inverted,
                Spindexer.maxDutyCycle,
                Spindexer.maxVoltage);
        if (!TalonFXUtils.safeApplyConfig(spindexerMotor, Spindexer.motorName, spindexerConfig)) {
            spindexerConfigFail.set(true);
        }

        exchangeMotor = new SparkMax(Exchange.motorID, MotorType.kBrushless);
        SparkMaxConfig exchangeConfig = new SparkMaxConfig();
        SparkUtils.configureBasicSettings(
                exchangeConfig,
                Exchange.maxCurrent,
                Exchange.idleMode,
                Exchange.inverted,
                Exchange.maxDutyCycle,
                Exchange.nominalVoltage);
        if (!SparkUtils.safeApplyConfig(exchangeMotor, Exchange.motorName, exchangeConfig)) {
            exchangeConfigFail.set(true);
        }
    }

    @Override
    public String getName() {
        return IndexerConstants.subsystemName;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(getName(), this);

        Logger.logTalonFX(getName(), Spindexer.motorName, spindexerMotor);

        Logger.logSparkMotor(getName(), Exchange.motorName, exchangeMotor);
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean spindexerConnected = spindexerMotor.isConnected();

        CANMonitor.logCANDeviceStatus(Spindexer.motorName, Spindexer.motorID, spindexerConnected);
        spindexerDisconnected.set(!spindexerConnected);
        spindexerOverheating.set(spindexerMotor.getDeviceTemp().getValue().in(Units.Celsius) >= 80);
        spindexerFaults.set(TalonFXUtils.getAllActiveFaults(spindexerMotor).hasCriticalFaults());

        boolean exchangeConnected = SparkUtils.isConnected(exchangeMotor);
        CANMonitor.logCANDeviceStatus(Exchange.motorName, Exchange.motorID, exchangeConnected);
        exchangeDisconnected.set(!exchangeConnected);
        exchangeOverheating.set(exchangeMotor.getMotorTemperature() >= 80);
        exchangeFaults.set(SparkUtils.hasCriticalFaults(exchangeMotor.getFaults()));
        exchangeWarnings.set(SparkUtils.hasCriticalWarnings(exchangeMotor.getWarnings()));
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
