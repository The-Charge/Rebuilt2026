package frc.robot.utils;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class TalonFXUtils {

    private TalonFXUtils() {}

    /**
     * WARNINGS: this is very slow, do not call it frequently
     */
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

    /**
     * WARNING: this is very slow, do not call it frequently
     */
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

    /**
     * Modify the given config to contain basic motor settings
     * @param config The {@code TalonFXConfiguration} to modify
     * @param maxAmps
     * @param neutralMode
     * @param inverted
     * @param maxDutyCycle
     * @param maxVoltage
     */
    public static void configureBasicSettings(
            TalonFXConfiguration config,
            Current maxAmps,
            NeutralModeValue neutralMode,
            InvertedValue inverted,
            double maxDutyCycle,
            Optional<Voltage> maxVoltage) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null TalonFXConfiguration", true);
            return;
        }

        config.CurrentLimits.StatorCurrentLimit = maxAmps.in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = neutralMode;

        config.MotorOutput.Inverted = inverted;

        config.MotorOutput.PeakForwardDutyCycle = maxDutyCycle;
        config.MotorOutput.PeakReverseDutyCycle = -maxDutyCycle;

        if (maxVoltage != null && maxVoltage.isPresent()) {
            config.Voltage.PeakForwardVoltage = maxVoltage.get().in(Volts);
            config.Voltage.PeakReverseVoltage = -maxVoltage.get().in(Volts);
        }
    }

    /**
     * Modify the given config to contain closed loop settings
     * @param config The {@code TalonFXConfiguration} to modify
     * @param kP
     * @param kI
     * @param kD
     * @param kG Unit depends on control mode
     * @param gravityType
     * @param kS Measured in units of output
     * @param kSSign
     * @param kV Measured in units of output/s
     * @param kA Measured in units of output/s^2
     */
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

    /**
     * Modify the given config to contain settings for Motion Magic
     * @param config The {@code TalonFXConfiguration} to modify
     * @param cruiseRPS
     * @param accelRPS2
     * @param jerkRPS3
     */
    public static void configureMotionMagicSettings(
            TalonFXConfiguration config, double cruiseRPS, double accelRPS2, Optional<Double> jerkRPS3) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null TalonFXConfiguration", true);
            return;
        }

        config.MotionMagic.MotionMagicCruiseVelocity = cruiseRPS;
        config.MotionMagic.MotionMagicAcceleration = accelRPS2;
        config.MotionMagic.MotionMagicJerk = jerkRPS3.orElse(0.0d);
    }

    /**
     * Modify the given config to contain settings for soft stops
     * @param config The {@code TalonFXConfiguration} to modify
     * @param forwardLimitRots
     * @param reverseLimitRots
     */
    public static void configureSoftStops(
            TalonFXConfiguration config, Optional<Double> forwardLimitRots, Optional<Double> reverseLimitRots) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null TalonFXConfiguration", true);
            return;
        }

        if (forwardLimitRots != null) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitRots.isPresent();
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimitRots.orElse(0d);
        }

        if (reverseLimitRots != null) {
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitRots.isPresent();
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimitRots.orElse(0d);
        }
    }

    /**
     * Modify the given config to contain settings for limit switches
     * @param config The {@code TalonFXConfiguration} to modify
     * @param forwardLimitEnabled
     * @param forwardLimitResetRots The value the encoder wil be reset to when the forward limit switch is reached. This does nothing if the forward limit is disabled
     * @param reverseLimitEnabled
     * @param reverseLimitResetRots The value the encoder wil be reset to when the reverse limit switch is reached. This does nothing if the reverse limit is disabled
     */
    public static void configureLimitSwitches(
            TalonFXConfiguration config,
            boolean forwardLimitEnabled,
            Optional<Double> forwardLimitResetRots,
            boolean reverseLimitEnabled,
            Optional<Double> reverseLimitResetRots) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null TalonFXConfiguration", true);
            return;
        }

        config.HardwareLimitSwitch.ForwardLimitEnable = forwardLimitEnabled;
        if (forwardLimitResetRots != null) {
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forwardLimitResetRots.isPresent();
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = forwardLimitResetRots.orElse(0d);
        }

        config.HardwareLimitSwitch.ReverseLimitEnable = reverseLimitEnabled;
        if (reverseLimitResetRots != null) {
            config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverseLimitResetRots.isPresent();
            config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = reverseLimitResetRots.orElse(0d);
        }
    }
}
