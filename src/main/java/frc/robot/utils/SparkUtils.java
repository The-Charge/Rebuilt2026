package frc.robot.utils;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import java.util.Optional;

public class SparkUtils {

    private SparkUtils() {}

    public static boolean hasCriticalFaults(Faults faults) {
        if (faults == null) return false;

        return faults.can
                || faults.escEeprom
                || faults.firmware
                || faults.gateDriver
                || faults.motorType
                || faults.other
                || faults.sensor
                || faults.temperature;
    }

    public static boolean hasCriticalWarnings(Warnings warnings) {
        if (warnings == null) return false;

        return warnings.brownout
                || warnings.escEeprom
                || warnings.extEeprom
                || warnings.other
                || warnings.overcurrent
                || warnings.sensor;
    }

    public static boolean isConnected(SparkBase motor) {
        if (motor == null) {
            Logger.reportWarning("Cannot test connectivity of a null motor", true);
            return false;
        }

        return motor.getFirmwareVersion() != 0;
    }

    public void configureBasicSettings(
            SparkBaseConfig config,
            int maxCurrent,
            IdleMode idleMode,
            boolean inverted,
            double maxDutyCycle,
            Optional<Double> voltageCompensation) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        config.smartCurrentLimit(maxCurrent);
        config.idleMode(idleMode);
        config.inverted(inverted);
        config.closedLoop.outputRange(-maxDutyCycle, maxDutyCycle);

        if (voltageCompensation != null && voltageCompensation.isPresent()) {
            config.voltageCompensation(voltageCompensation.get());
        } else {
            config.disableVoltageCompensation();
        }
    }

    /**
     * @param config
     * @param kP
     * @param kI
     * @param kD
     * @param kStaticG this parameter should be {@code Optional.empty()} if {@code kCos} is not zero
     * @param kCos this parameter should be {@code Optional.empty()} if {@code kStaticG} is not zero
     * @param kS
     * @param kV
     * @param kA
     */
    public void configureClosedLoopSettings(
            SparkBaseConfig config,
            double kP,
            double kI,
            double kD,
            Optional<Double> kStaticG,
            Optional<Double> kCos,
            Optional<Double> kS,
            Optional<Double> kV,
            Optional<Double> kA) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        config.closedLoop.pid(kP, kI, kD);
        if (kStaticG != null && kStaticG.isPresent()) {
            config.closedLoop.feedForward.kG(kStaticG.get());
        }
        if (kCos != null && kCos.isPresent()) {
            config.closedLoop.feedForward.kCos(kCos.get());
        }
        if (kS != null && kS.isPresent()) {
            config.closedLoop.feedForward.kS(kS.get());
        }
        if (kV != null && kV.isPresent()) {
            config.closedLoop.feedForward.kV(kV.get());
        }
        if (kA != null && kA.isPresent()) {
            config.closedLoop.feedForward.kA(kA.get());
        }
    }

    public void configureSoftStops(
            SparkBaseConfig config, Optional<Double> forwardLimit, Optional<Double> reverseLimit) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        if (forwardLimit != null) {
            config.softLimit.forwardSoftLimitEnabled(forwardLimit.isPresent());
            config.softLimit.forwardSoftLimit(forwardLimit.orElse(0d));
        }

        if (reverseLimit != null) {
            config.softLimit.reverseSoftLimitEnabled(reverseLimit.isPresent());
            config.softLimit.reverseSoftLimit(reverseLimit.orElse(0d));
        }
    }

    public void configureHardStops(
            SparkBaseConfig config,
            boolean forwardLimitEnabled,
            Optional<Double> forwardLimitResetValue,
            boolean reverseLimitEnabled,
            Optional<Double> reverseLimitResetValue) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        if (forwardLimitEnabled) {
            if (forwardLimitResetValue != null && forwardLimitResetValue.isPresent()) {
                config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
                config.limitSwitch.forwardLimitSwitchPosition(forwardLimitResetValue.get());
            } else {
                config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
            }
        } else {
            config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        }

        if (reverseLimitEnabled) {
            if (reverseLimitResetValue != null && reverseLimitResetValue.isPresent()) {
                config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
                config.limitSwitch.reverseLimitSwitchPosition(reverseLimitResetValue.get());
            } else {
                config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
            }
        } else {
            config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        }
    }
}
