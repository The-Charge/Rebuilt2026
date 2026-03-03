package frc.robot.utils;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Voltage;
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

    public static void configureBasicSettings(
            SparkBaseConfig config,
            int maxAmps,
            IdleMode idleMode,
            boolean inverted,
            double maxDutyCycle,
            Optional<Double> nominalVoltage) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        config.smartCurrentLimit(maxAmps);
        config.idleMode(idleMode);
        config.inverted(inverted);
        config.closedLoop.outputRange(-maxDutyCycle, maxDutyCycle);

        if (nominalVoltage != null && nominalVoltage.isPresent()) {
            config.voltageCompensation(nominalVoltage.get());
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
    public static void configureClosedLoopSettings(
            SparkBaseConfig config,
            double kP,
            double kI,
            double kD,
            Optional<Voltage> kStaticG,
            Optional<Voltage> kCos,
            Optional<Voltage> kS,
            Optional<Voltage> kV,
            Optional<Voltage> kA,
            Optional<Double> iZone) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        config.closedLoop.pid(kP, kI, kD);
        if (kStaticG != null && kStaticG.isPresent()) {
            config.closedLoop.feedForward.kG(kStaticG.get().in(Volts));
        }
        if (kCos != null && kCos.isPresent()) {
            config.closedLoop.feedForward.kCos(kCos.get().in(Volts));
        }
        if (kS != null && kS.isPresent()) {
            config.closedLoop.feedForward.kS(kS.get().in(Volts));
        }
        if (kV != null && kV.isPresent()) {
            config.closedLoop.feedForward.kV(kV.get().in(Volts));
        }
        if (kA != null && kA.isPresent()) {
            config.closedLoop.feedForward.kA(kA.get().in(Volts));
        }
        if (iZone != null && iZone.isPresent()) {
            config.closedLoop.iZone(iZone.get());
        }
    }

    public static void configureSoftStops(
            SparkBaseConfig config, Optional<Double> forwardLimitRots, Optional<Double> reverseLimitRots) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        if (forwardLimitRots != null) {
            config.softLimit.forwardSoftLimitEnabled(forwardLimitRots.isPresent());
            config.softLimit.forwardSoftLimit(forwardLimitRots.orElse(0d));
        }

        if (reverseLimitRots != null) {
            config.softLimit.reverseSoftLimitEnabled(reverseLimitRots.isPresent());
            config.softLimit.reverseSoftLimit(reverseLimitRots.orElse(0d));
        }
    }

    public static void configureHardStops(
            SparkBaseConfig config,
            boolean forwardLimitEnabled,
            Optional<Double> forwardLimitResetRots,
            boolean reverseLimitEnabled,
            Optional<Double> reverseLimitResetRots) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        if (forwardLimitEnabled) {
            if (forwardLimitResetRots != null && forwardLimitResetRots.isPresent()) {
                config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
                config.limitSwitch.forwardLimitSwitchPosition(forwardLimitResetRots.get());
            } else {
                config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
            }
        } else {
            config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        }

        if (reverseLimitEnabled) {
            if (reverseLimitResetRots != null && reverseLimitResetRots.isPresent()) {
                config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
                config.limitSwitch.reverseLimitSwitchPosition(reverseLimitResetRots.get());
            } else {
                config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
            }
        } else {
            config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        }
    }
}
