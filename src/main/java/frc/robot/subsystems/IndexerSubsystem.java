package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexConstants;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX spindexerMotor; // MAKES MOTOR

    public IndexerSubsystem() {

        spindexerMotor = new TalonFX(IndexConstants.spindexerMotorID); // port number under indexConstants

        configureTalonFXMotor(); // always configure before their use
    }

    public double getVelocityRPM() {
        return spindexerMotor.getVelocity().getValue().abs(Units.RPM); // to check velocity for spinup
    }

    public void setIndexerVelocity(double velocity) {
        VelocityVoltage request = new VelocityVoltage(velocity);
        spindexerMotor.setControl(request); // says that velocity controls velocity
    }

    @Override
    public void periodic() {}

    public void stop() {
        spindexerMotor.stopMotor(); // safety
    }

    private void configureTalonFXMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // current limits
        config.CurrentLimits.StatorCurrentLimit = IndexConstants.maxCurrent;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // hardstops
        config.HardwareLimitSwitch.ForwardLimitEnable = IndexConstants.forwardHardLimitEnabled;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable =
                IndexConstants.forwardHardLimitResetValue.isPresent();
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue =
                IndexConstants.forwardHardLimitResetValue.orElse(0d);
        config.HardwareLimitSwitch.ReverseLimitEnable = IndexConstants.reverseHardLimitEnabled;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable =
                IndexConstants.reverseHardLimitResetValue.isPresent();
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue =
                IndexConstants.reverseHardLimitResetValue.orElse(0d);

        // braking mode
        config.MotorOutput.NeutralMode = IndexConstants.neutralMode;
        // polarity
        config.MotorOutput.Inverted = IndexConstants.inverted;
        // duty cycle output limits
        config.MotorOutput.PeakForwardDutyCycle = IndexConstants.maxDutyCycle;
        config.MotorOutput.PeakReverseDutyCycle = -IndexConstants.maxDutyCycle;

        // pid
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        config.Slot0.kP = IndexConstants.kP;
        config.Slot0.kI = IndexConstants.kI;
        config.Slot0.kD = IndexConstants.kD;
        config.Slot0.kG = IndexConstants.kG;

        // softstops
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = IndexConstants.forwardSoftLimit.isPresent();
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IndexConstants.forwardSoftLimit.orElse(0d);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = IndexConstants.reverseSoftLimit.isPresent();
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IndexConstants.reverseSoftLimit.orElse(0d);

        // voltage output limits
        config.Voltage.PeakForwardVoltage = IndexConstants.maxVoltage;
        config.Voltage.PeakReverseVoltage = -IndexConstants.maxVoltage;

        if (spindexerMotor.getConfigurator().apply(config) != StatusCode.OK) {
            // TODO: error reporting
            // Logger.reportError("Failed to configure elevator motor");
            // Alerts.elevMotorConfigFail.set(true);
        }
    }
}
