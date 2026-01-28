package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import java.util.Optional;

public class TalonFXUtils {

    private TalonFXUtils() {}

    public static TalonFXFaults getAllActiveFaults(TalonFX motor) {
        if (motor == null) {
            Logger.reportWarning("Cannot get faults from a null TalonFX", true);
            return new TalonFXFaults(
                    false, false, false, false, false, false, false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false, false, false);
        }

        return new TalonFXFaults(
                motor.getFault_BootDuringEnable().getValue(),
                motor.getFault_BridgeBrownout().getValue(),
                motor.getFault_DeviceTemp().getValue(),
                motor.getFault_ForwardHardLimit().getValue(),
                motor.getFault_ForwardSoftLimit().getValue(),
                motor.getFault_FusedSensorOutOfSync().getValue(),
                motor.getFault_Hardware().getValue(),
                motor.getFault_MissingDifferentialFX().getValue(),
                motor.getFault_MissingHardLimitRemote().getValue(),
                motor.getFault_MissingSoftLimitRemote().getValue(),
                motor.getFault_OverSupplyV().getValue(),
                motor.getFault_ProcTemp().getValue(),
                motor.getFault_RemoteSensorDataInvalid().getValue(),
                motor.getFault_RemoteSensorPosOverflow().getValue(),
                motor.getFault_RemoteSensorReset().getValue(),
                motor.getFault_ReverseHardLimit().getValue(),
                motor.getFault_ReverseSoftLimit().getValue(),
                motor.getFault_StaticBrakeDisabled().getValue(),
                motor.getFault_StatorCurrLimit().getValue(),
                motor.getFault_SupplyCurrLimit().getValue(),
                motor.getFault_Undervoltage().getValue(),
                motor.getFault_UnlicensedFeatureInUse().getValue(),
                motor.getFault_UnstableSupplyV().getValue(),
                motor.getFault_UsingFusedCANcoderWhileUnlicensed().getValue());
    }

    public static TalonFXFaults getAllStickyFaults(TalonFX motor) {
        if (motor == null) {
            Logger.reportWarning("Cannot get sticky faults from a null TalonFX", true);
            return new TalonFXFaults(
                    false, false, false, false, false, false, false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false, false, false);
        }

        return new TalonFXFaults(
                motor.getStickyFault_BootDuringEnable().getValue(),
                motor.getStickyFault_BridgeBrownout().getValue(),
                motor.getStickyFault_DeviceTemp().getValue(),
                motor.getStickyFault_ForwardHardLimit().getValue(),
                motor.getStickyFault_ForwardSoftLimit().getValue(),
                motor.getStickyFault_FusedSensorOutOfSync().getValue(),
                motor.getStickyFault_Hardware().getValue(),
                motor.getStickyFault_MissingDifferentialFX().getValue(),
                motor.getStickyFault_MissingHardLimitRemote().getValue(),
                motor.getStickyFault_MissingSoftLimitRemote().getValue(),
                motor.getStickyFault_OverSupplyV().getValue(),
                motor.getStickyFault_ProcTemp().getValue(),
                motor.getStickyFault_RemoteSensorDataInvalid().getValue(),
                motor.getStickyFault_RemoteSensorPosOverflow().getValue(),
                motor.getStickyFault_RemoteSensorReset().getValue(),
                motor.getStickyFault_ReverseHardLimit().getValue(),
                motor.getStickyFault_ReverseSoftLimit().getValue(),
                motor.getStickyFault_StaticBrakeDisabled().getValue(),
                motor.getStickyFault_StatorCurrLimit().getValue(),
                motor.getStickyFault_SupplyCurrLimit().getValue(),
                motor.getStickyFault_Undervoltage().getValue(),
                motor.getStickyFault_UnlicensedFeatureInUse().getValue(),
                motor.getStickyFault_UnstableSupplyV().getValue(),
                motor.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue());
    }

    public static void configureBasicSettings(
            TalonFXConfiguration config,
            double maxCurrent,
            NeutralModeValue neutralMode,
            InvertedValue inverted,
            double maxDutyCycle,
            Optional<Double> maxVoltage) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null TalonFXConfiguration", true);
            return;
        }

        config.CurrentLimits.StatorCurrentLimit = maxCurrent;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = neutralMode;

        config.MotorOutput.Inverted = inverted;

        config.MotorOutput.PeakForwardDutyCycle = maxDutyCycle;
        config.MotorOutput.PeakReverseDutyCycle = -maxDutyCycle;

        if (maxVoltage != null && maxVoltage.isPresent()) {
            config.Voltage.PeakForwardVoltage = maxVoltage.get();
            config.Voltage.PeakReverseVoltage = -maxVoltage.get();
        }
    }

    public static void configureClosedLoopSettings(
            TalonFXConfiguration config,
            double kP,
            double kI,
            double kD,
            Optional<Double> kG,
            Optional<GravityTypeValue> gravityType,
            Optional<Double> kS,
            Optional<StaticFeedforwardSignValue> kSSign,
            Optional<Double> kV,
            Optional<Double> kA) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null TalonFXConfiguration", true);
            return;
        }

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        if (kG != null && kG.isPresent()) {
            config.Slot0.kG = kG.get();
        }
        if (gravityType != null && gravityType.isPresent()) {
            config.Slot0.GravityType = gravityType.get();
        }
        if (kS != null && kS.isPresent()) {
            config.Slot0.kS = kS.get();
        }
        if (kSSign != null && kSSign.isPresent()) {
            config.Slot0.StaticFeedforwardSign = kSSign.get();
        }
        if (kV != null && kV.isPresent()) {
            config.Slot0.kV = kV.get();
        }
        if (kA != null && kA.isPresent()) {
            config.Slot0.kA = kA.get();
        }
    }

    public static void configureSoftStops(
            TalonFXConfiguration config, Optional<Double> forwardLimit, Optional<Double> reverseLimit) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null TalonFXConfiguration", true);
            return;
        }

        if (forwardLimit != null) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimit.isPresent();
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit.orElse(0d);
        }

        if (reverseLimit != null) {
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimit.isPresent();
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit.orElse(0d);
        }
    }

    public static void configureHardStops(
            TalonFXConfiguration config,
            boolean forwardLimitEnabled,
            Optional<Double> forwardLimitResetValue,
            boolean reverseLimitEnabled,
            Optional<Double> reverseLimitResetValue) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null TalonFXConfiguration", true);
            return;
        }

        config.HardwareLimitSwitch.ForwardLimitEnable = forwardLimitEnabled;
        if (forwardLimitResetValue != null) {
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forwardLimitResetValue.isPresent();
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = forwardLimitResetValue.orElse(0d);
        }

        config.HardwareLimitSwitch.ReverseLimitEnable = reverseLimitEnabled;
        if (reverseLimitResetValue != null) {
            config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverseLimitResetValue.isPresent();
            config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = reverseLimitResetValue.orElse(0d);
        }
    }
}
