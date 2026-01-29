package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexConstants;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import frc.robot.utils.TalonFXUtils;
import java.util.Optional;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX spindexerMotor; // MAKES MOTOR
    private final SparkMax gateMotor;

    public IndexerSubsystem() {

        spindexerMotor = new TalonFX(IndexConstants.Spindexer.motorID); // port number under indexConstants
        TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
        TalonFXUtils.configureBasicSettings(
                spindexerConfig,
                IndexConstants.Spindexer.maxCurrent,
                IndexConstants.Spindexer.neutralMode,
                IndexConstants.Spindexer.inverted,
                IndexConstants.Spindexer.maxDutyCycle,
                Optional.of(IndexConstants.Spindexer.maxVoltage));
        TalonFXUtils.configureClosedLoopSettings(
                spindexerConfig,
                IndexConstants.Spindexer.kP,
                IndexConstants.Spindexer.kI,
                IndexConstants.Spindexer.kD,
                Optional.empty(),
                Optional.empty());
        spindexerMotor.getConfigurator().apply(spindexerConfig); // TODO: check config apply

        gateMotor = new SparkMax(
                IndexConstants.Gate.motorID,
                MotorType.kBrushless); // port number in IndexConstants; defines the motor as brushless
        SparkMaxConfig gateConfig = new SparkMaxConfig();
        SparkUtils.configureBasicSettings(
                gateConfig,
                IndexConstants.Gate.maxCurrent,
                IndexConstants.Gate.idleMode,
                IndexConstants.Gate.inverted,
                IndexConstants.Gate.maxDutyCycle,
                IndexConstants.Gate.nominalVoltage);
        gateMotor.configure(gateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        Logger.logSubsystem("Indexer", this);
        Logger.logTalonFX("Indexer", "spindexerMotor", spindexerMotor, Optional.empty());
        Logger.logSparkMotor("Indexer", "gateMotor", gateMotor, Optional.empty());
    }

    public double getSpindexerRPM() {
        return spindexerMotor.getVelocity().getValue().abs(Units.RPM); // to check velocity for spinup
    }

    public void setSpindexerMotorVelocity(double RPM) {
        VelocityVoltage request = new VelocityVoltage(RPM);
        spindexerMotor.setControl(request); // says that velocity controls velocity
    }

    public void setGateMotorVoltage(double voltage) {
        gateMotor.setVoltage(voltage);
    }

    public void stop() {
        spindexerMotor.stopMotor(); // safety
        gateMotor.stopMotor();
    }
}
