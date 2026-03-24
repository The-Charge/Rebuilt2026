package frc.robot.utils;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
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

        return warnings.escEeprom || warnings.extEeprom || warnings.other || warnings.overcurrent || warnings.sensor;
    }

    /**
     * WARNING: this is pretty slow, do not call it frequently
     */
    public static boolean isConnected(SparkBase motor) {
        if (motor == null) {
            Logger.reportWarning("Cannot test connectivity of a null motor", true);
            return false;
        }

        return motor.getFirmwareVersion() != 0;
    }

    /**
     * Modify the given config to contain some basic motor settings
     * @param config The {@code SparkBaseConfig} to modify
     * @param maxCurrent
     * @param idleMode
     * @param inverted
     * @param maxDutyCycle Maximum percent output in either direction, should be in range [0, 1]
     * @param nominalVoltage If provided, will enable voltage compensation. See {@see <a href="https://www.chiefdelphi.com/t/behavior-of-sparkmax-voltage-compensation/459782/2">this post</a>} for a better explanation than I can give
     */
    public static void configureBasicSettings(
            SparkBaseConfig config,
            Current maxCurrent,
            IdleMode idleMode,
            boolean inverted,
            double maxDutyCycle,
            Optional<Voltage> nominalVoltage) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        config.smartCurrentLimit((int) maxCurrent.in(Amps));
        config.idleMode(idleMode);
        config.inverted(inverted);
        config.closedLoop.outputRange(-maxDutyCycle, maxDutyCycle);

        if (nominalVoltage != null && nominalVoltage.isPresent()) {
            config.voltageCompensation(nominalVoltage.get().in(Volts));
        } else {
            config.disableVoltageCompensation();
        }
    }

    /**
     * Modify the given config to contain settings for closed loop control. More info: {@see <a href="https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control#feed-forward-constant-terms">how feed forwards apply</a>}, {@see <a href="https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control#feed-forward-constant-quick-reference">feed forward units</a>}
     * @param config The {@code SparkBaseConfig} to modify
     * @param kP
     * @param kI
     * @param iZone The range in which the Integral component will accumulate. {@see <a href="https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/ProfiledPIDController.html#setIZone(double)">Here's a better explantion</a>}
     * @param kD
     * @param kStaticG This parameter should be {@code Optional.empty()} if {@code kCos} is not {@code Optional.empty()}
     * @param kCos This parameter should be {@code Optional.empty()} if {@code kStaticG} is not {@code Optional.empty()}
     * @param kS The amount of voltage required to overcome friction (feedforward). Measured in Volts
     * @param kV The amount of voltage applied per target velocity. Measured in Volts/RPM
     * @param kA The amount of voltage applied per target acceleration. Measured in Volts/RPM/s
     * @param rampTime The minimum amount of time allowed to go from neutral to 100% output *NOTE: the documenation is very questionable, this may be incorrect
     */
    public static void configureClosedLoopSettings(
            SparkBaseConfig config,
            double kP,
            double kI,
            Optional<Double> iZone,
            double kD,
            Optional<Voltage> kStaticG,
            Optional<Voltage> kCos,
            Optional<Double> kS,
            Optional<Double> kV,
            Optional<Double> kA,
            Optional<Time> rampTime) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        config.closedLoop.pid(kP, kI, kD);
        if (iZone != null && iZone.isPresent()) {
            config.closedLoop.iZone(iZone.get());
        }
        if (kStaticG != null && kStaticG.isPresent()) {
            config.closedLoop.feedForward.kG(kStaticG.get().in(Volts));
        }
        if (kCos != null && kCos.isPresent()) {
            config.closedLoop.feedForward.kCos(kCos.get().in(Volts));
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
        if (rampTime != null && rampTime.isPresent()) {
            config.closedLoopRampRate(rampTime.get().in(Seconds));
        }
    }

    /**
     * Modify the given config to contain soft stop settings
     * @param config The {@code SparkBaseConfig} to modify
     * @param forwardLimitRots
     * @param reverseLimitRots
     */
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

    /**
     * Modify the given config to contain limit switch settings
     * @param config The {@code SparkBaseConfig} to modify
     * @param forwardLimitEnabled
     * @param forwardLimitResetRots The value the encoder will be reset to when the forward limit is hit. Does nothing if the forward limit is disabled
     * @param reverseLimitEnabled
     * @param reverseLimitResetRots The value the encoder will be reset to when the reverse limit is hit. Does nothing if the reverse limit is disabled
     */
    public static void configureLimitSwitches(
            SparkBaseConfig config,
            boolean forwardLimitEnabled,
            Optional<Angle> forwardLimitResetRots,
            boolean reverseLimitEnabled,
            Optional<Angle> reverseLimitResetRots) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        if (forwardLimitEnabled) {
            if (forwardLimitResetRots != null && forwardLimitResetRots.isPresent()) {
                config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
                config.limitSwitch.forwardLimitSwitchPosition(
                        forwardLimitResetRots.get().in(Rotations));
            } else {
                config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
            }
        } else {
            config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        }

        if (reverseLimitEnabled) {
            if (reverseLimitResetRots != null && reverseLimitResetRots.isPresent()) {
                config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
                config.limitSwitch.reverseLimitSwitchPosition(
                        reverseLimitResetRots.get().in(Rotations));
            } else {
                config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);
            }
        } else {
            config.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
        }
    }

    /**
     * Modify the given config to contain settings for MAXMotion control
     * @param config The {@code SparkBaseConfig} to modify
     * @param maxAccel Maximum acceleration and deceleration rate of the motor
     * @param cruiseVel Maximum velocity. This is not necessary if you are only going to be using MAXMotion velocity control
     * @param allowedError Maximum allowed error before profile regenerates itself. This is only used for MAXMotion position control
     */
    public static void configureMAXMotion(
            SparkBaseConfig config,
            AngularAcceleration maxAccel,
            Optional<AngularVelocity> cruiseVel,
            Optional<Angle> allowedError) {
        if (config == null) {
            Logger.reportWarning("Cannot modify a null SparkBaseConfig", true);
            return;
        }

        config.closedLoop.maxMotion.maxAcceleration(maxAccel.in(RotationsPerSecondPerSecond)
                * 60); // wants r/(ms), but units class only has r/s^2, so just multiply by 60 to get r/(ms)
        if (cruiseVel != null && cruiseVel.isPresent()) {
            config.closedLoop.maxMotion.cruiseVelocity(cruiseVel.get().in(RPM));
        }
        if (allowedError != null && allowedError.isPresent()) {
            config.closedLoop.maxMotion.allowedProfileError(allowedError.get().in(Rotations));
        }
    }
}
