package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;

public class Logger {

    private static final String PREFIX = "[Logger] ";

    private static boolean hasInited;
    private static NetworkTableInstance ntInstance;
    private static NetworkTable nt;

    static {
        hasInited = false;
    }

    private Logger() {}

    public static void init() {
        if (hasInited) return;

        DataLogManager.start();
        DataLogManager.logConsoleOutput(true);
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        ntInstance = NetworkTableInstance.getDefault();
        nt = ntInstance.getTable(""); // root table

        println("Logging started");

        hasInited = true;
    }

    /**
     * Use instead of System.out.println
     */
    public static void println(String msg) {
        if (msg == null) return; // silently quit
        DataLogManager.log(PREFIX + msg);
    }

    /**
     * Report a warning and log it to the console
     */
    public static void reportWarning(String msg, boolean printFullTrace) {
        reportWarning(msg, generateStackTrace(Thread.currentThread().getStackTrace(), 2), printFullTrace);
    }

    /**
     * Report a warning and log it to the console
     */
    public static void reportWarning(Exception e, boolean printFullTrace) {
        if (e == null) {
            reportWarning("Unknown warning exception, attempted to log null Exception", true);
        } else {
            reportWarning(e.getMessage(), generateStackTrace(e.getStackTrace(), 0), printFullTrace);
        }
    }

    private static void reportWarning(String msg, StackTrace trace, boolean printFullTrace) {
        // safe because of short circuit logic evalutation
        if (msg == null || msg.isEmpty()) msg = "No message provided";
        if (trace == null) {
            trace = new StackTrace();
            trace.location = "Invalid StackTrace";
            trace.trace = "Invalid StackTrace";
        }
        if (trace.location == null) {
            trace.location = "Invalid StackTrace.location";
        }
        if (trace.trace == null) {
            trace.trace = "Invalid StackTrace.trace";
        }

        StringBuilder builder = new StringBuilder();
        builder.append("Warning at ");
        builder.append(trace.location);
        builder.append(": ");
        builder.append(msg);

        if (printFullTrace) {
            builder.append('\n');
            builder.append(trace.trace);
        }

        println(builder.toString());
    }

    /**
     * Report an error and log it to the console
     */
    public static void reportError(String msg) {
        reportError(msg, generateStackTrace(Thread.currentThread().getStackTrace(), 2));
    }

    /**
     * Report an error and log it to the console
     */
    public static void reportError(Exception e) {
        if (e == null) {
            reportError("Unknown error exception, attempted to log null Exception");
        } else {
            reportError(e.getMessage(), generateStackTrace(e.getStackTrace(), 0));
        }
    }

    private static void reportError(String msg, StackTrace trace) {
        if (msg == null || msg.isEmpty()) msg = "No message provided";
        if (trace == null) {
            trace = new StackTrace();
            trace.location = "Invalid StackTrace";
            trace.trace = "Invalid StackTrace";
        }
        if (trace.location == null) {
            trace.location = "Invalid StackTrace.location";
        }
        if (trace.trace == null) {
            trace.trace = "Invalid StackTrace.trace";
        }

        StringBuilder builder = new StringBuilder();
        builder.append("ERROR at ");
        builder.append(trace.location);
        builder.append(": ");
        builder.append(msg);
        builder.append('\n');
        builder.append(trace.trace);

        println(builder.toString());
    }

    public static void logBool(String subsystem, String key, boolean val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setBoolean(val)) {
            reportWarning(
                    "attempted to log boolean value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logBoolArray(String subsystem, String key, boolean[] val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null boolean[]", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setBooleanArray(val)) {
            reportWarning(
                    "attempted to log boolean[] value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logDouble(String subsystem, String key, double val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setDouble(val)) {
            reportWarning(
                    "attempted to log double value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logDoubleArray(String subsystem, String key, double[] val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null double[]", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setDoubleArray(val)) {
            reportWarning(
                    "attempted to log double[] value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logFloat(String subsystem, String key, float val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setFloat(val)) {
            reportWarning(
                    "attempted to log float value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logFloatArray(String subsystem, String key, float[] val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null float[]", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setFloatArray(val)) {
            reportWarning(
                    "attempted to log float[] value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logLong(String subsystem, String key, long val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setInteger(val)) {
            reportWarning(
                    "attempted to log int value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logLongArray(String subsystem, String key, long[] val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null long[]", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setIntegerArray(val)) {
            reportWarning(
                    "attempted to log int[] value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logRaw(String subsystem, String key, ByteBuffer val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null ByteBuffer", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setRaw(val)) {
            reportWarning(
                    "attempted to log raw value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logRaw(String subsystem, String key, byte[] val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null byte[]", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setRaw(val)) {
            reportWarning(
                    "attempted to log raw value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logString(String subsystem, String key, String val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) val = "";

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setString(val)) {
            reportWarning(
                    "attempted to log String value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logStringArray(String subsystem, String key, String[] val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null String[]", true);
            return;
        }

        String normalized = NetworkTable.normalizeKey(subsystem + "/" + key, false);
        if (!nt.getEntry(normalized).setStringArray(val)) {
            reportWarning(
                    "attempted to log String[] value to entry '" + key + "' of type "
                            + nt.getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logEnum(String subsystem, String key, Enum<?> val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null Enum", true);
            return;
        }

        logString(subsystem, key, val.name());
    }

    public static void logEnumArray(String subsystem, String key, Enum<?>[] val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null Enum[]", true);
            return;
        }

        logStringArray(subsystem, key, (String[])
                Arrays.stream(val).map((Enum<?> i) -> i == null ? "" : i.name()).toArray());
    }

    public static void logTalonFX(
            String subsystem, String name, TalonFX motor, Optional<Map<String, String>> additionalData) {
        if (subsystem == null) subsystem = "";
        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return;
        }
        if (motor == null) {
            reportWarning("Cannot log a null TalonFX", true);
            return;
        }

        String table = subsystem + "/" + name;

        TalonFXFaults activeFaults = TalonFXUtils.getAllActiveFaults(motor);
        TalonFXFaults stickyFaults = TalonFXUtils.getAllStickyFaults(motor);

        logDouble(table, "positionRots", motor.getPosition().getValue().abs(Units.Rotations));
        logDouble(table, "velocityRPM", motor.getVelocity().getValue().abs(Units.RPM));
        logDouble(table, "tempC", motor.getDeviceTemp().getValue().abs(Units.Celsius));
        logDouble(table, "dutyCycle", motor.get());
        logDouble(table, "voltageOut", motor.getMotorVoltage().getValue().abs(Units.Volts));
        logDouble(table, "voltageIn", motor.getSupplyVoltage().getValue().abs(Units.Volts));
        logBool(table, "hardStopForward", activeFaults.forwardHardLimit());
        logBool(table, "hardStopReverse", activeFaults.reverseHardLimit());
        logBool(table, "softStopForward", activeFaults.forwardSoftLimit());
        logBool(table, "softStopReverse", activeFaults.reverseSoftLimit());
        logDouble(table, "currentOut", motor.getStatorCurrent().getValue().abs(Units.Amps));
        logDouble(table, "currentIn", motor.getSupplyCurrent().getValue().abs(Units.Amps));
        logEnum(table, "controlMode", motor.getControlMode().getValue());
        logDouble(table, "targetRots", motor.getClosedLoopReference().getValue().doubleValue());
        logBool(table, "enabled", motor.getDeviceEnable().getValue() == DeviceEnableValue.Enabled);
        logBool(table, "connected", motor.isConnected());
        logBool(table, "alive", motor.isAlive());

        logBool(table + "/faults", "bootDuringEnable", activeFaults.bootDuringEnable());
        logBool(table + "/faults", "bridgeBrownout", activeFaults.bridgeBrownout());
        logBool(table + "/faults", "deviceTemp", activeFaults.deviceTemp());
        logBool(table + "/faults", "forwardHardLimit", activeFaults.forwardHardLimit());
        logBool(table + "/faults", "fowardSoftLimit", activeFaults.forwardSoftLimit());
        logBool(table + "/faults", "fusedSensorOutOfSync", activeFaults.fusedSensorOutOfSync());
        logBool(table + "/faults", "hardware", activeFaults.hardware());
        logBool(table + "/faults", "missingDifferentialFX", activeFaults.missingDifferentialFX());
        logBool(table + "/faults", "missingHardLimitRemote", activeFaults.missingHardLimitRemote());
        logBool(table + "/faults", "missingSoftLimitRemote", activeFaults.missingSoftLimitRemote());
        logBool(table + "/faults", "overSupplyV", activeFaults.overSupplyV());
        logBool(table + "/faults", "procTemp", activeFaults.procTemp());
        logBool(table + "/faults", "remoteSensorDataInvalid", activeFaults.remoteSensorDataInvalid());
        logBool(table + "/faults", "remoteSensorPosOverflow", activeFaults.remoteSensorPosOverflow());
        logBool(table + "/faults", "remoteSensorReset", activeFaults.remoteSensorReset());
        logBool(table + "/faults", "reverseHardLimit", activeFaults.reverseHardLimit());
        logBool(table + "/faults", "reverseSoftLimit", activeFaults.reverseSoftLimit());
        logBool(table + "/faults", "staticBrakeDisabled", activeFaults.staticBrakeDisabled());
        logBool(table + "/faults", "statorCurrLimit", activeFaults.statorCurrLimit());
        logBool(table + "/faults", "supplyCurrLimit", activeFaults.supplyCurrLimit());
        logBool(table + "/faults", "undervoltage", activeFaults.undervoltage());
        logBool(table + "/faults", "unlicensedFeatureInUse", activeFaults.unlicensedFeatureInUse());
        logBool(table + "/faults", "unstableSupplyV", activeFaults.unstableSupplyV());
        logBool(
                table + "/faults",
                "usingFusedCANCoderWhileUnlicensed",
                activeFaults.usingFusedCANCoderWhileUnlicensed());
        logBool(table, "criticalFaultsActive", activeFaults.hasCriticalFaults());

        logBool(table + "/stickyFaults", "bootDuringEnable", stickyFaults.bootDuringEnable());
        logBool(table + "/stickyFaults", "bridgeBrownout", stickyFaults.bridgeBrownout());
        logBool(table + "/stickyFaults", "deviceTemp", stickyFaults.deviceTemp());
        logBool(table + "/stickyFaults", "forwardHardLimit", stickyFaults.forwardHardLimit());
        logBool(table + "/stickyFaults", "fowardSoftLimit", stickyFaults.forwardSoftLimit());
        logBool(table + "/stickyFaults", "fusedSensorOutOfSync", stickyFaults.fusedSensorOutOfSync());
        logBool(table + "/stickyFaults", "hardware", stickyFaults.hardware());
        logBool(table + "/stickyFaults", "missingDifferentialFX", stickyFaults.missingDifferentialFX());
        logBool(table + "/stickyFaults", "missingHardLimitRemote", stickyFaults.missingHardLimitRemote());
        logBool(table + "/stickyFaults", "missingSoftLimitRemote", stickyFaults.missingSoftLimitRemote());
        logBool(table + "/stickyFaults", "overSupplyV", stickyFaults.overSupplyV());
        logBool(table + "/stickyFaults", "procTemp", stickyFaults.procTemp());
        logBool(table + "/stickyFaults", "remoteSensorDataInvalid", stickyFaults.remoteSensorDataInvalid());
        logBool(table + "/stickyFaults", "remoteSensorPosOverflow", stickyFaults.remoteSensorPosOverflow());
        logBool(table + "/stickyFaults", "remoteSensorReset", stickyFaults.remoteSensorReset());
        logBool(table + "/stickyFaults", "reverseHardLimit", stickyFaults.reverseHardLimit());
        logBool(table + "/stickyFaults", "reverseSoftLimit", stickyFaults.reverseSoftLimit());
        logBool(table + "/stickyFaults", "staticBrakeDisabled", stickyFaults.staticBrakeDisabled());
        logBool(table + "/stickyFaults", "statorCurrLimit", stickyFaults.statorCurrLimit());
        logBool(table + "/stickyFaults", "supplyCurrLimit", stickyFaults.supplyCurrLimit());
        logBool(table + "/stickyFaults", "undervoltage", stickyFaults.undervoltage());
        logBool(table + "/stickyFaults", "unlicensedFeatureInUse", stickyFaults.unlicensedFeatureInUse());
        logBool(table + "/stickyFaults", "unstableSupplyV", stickyFaults.unstableSupplyV());
        logBool(
                table + "/stickyFaults",
                "usingFusedCANCoderWhileUnlicensed",
                stickyFaults.usingFusedCANCoderWhileUnlicensed());
        logBool(table, "criticalStickyFaultsActive", stickyFaults.hasCriticalFaults());

        if (additionalData != null && additionalData.isPresent()) {
            for (Entry<String, String> data : additionalData.get().entrySet()) {
                logString(table, data.getKey(), data.getValue());
            }
        }
    }

    public static void logSparkMotor(
            String subsystem, String name, SparkBase motor, Optional<Map<String, String>> additionalData) {
        if (subsystem == null) subsystem = "";
        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return;
        }
        if (motor == null) {
            reportWarning("Cannot log a null SparkMax", true);
            return;
        }

        String table = subsystem + "/" + name;

        Faults activeFaults = motor.getFaults();
        Faults stickyFaults = motor.getStickyFaults();
        Warnings activeWarnings = motor.getWarnings();
        Warnings stickyWarnings = motor.getStickyWarnings();

        logDouble(table, "positionRots", motor.getEncoder().getPosition());
        logDouble(table, "velocityRPM", motor.getEncoder().getVelocity());
        logDouble(table, "tempC", motor.getMotorTemperature());
        logDouble(table, "dutyCycle", motor.getAppliedOutput());
        logDouble(table, "voltageOut", motor.getAppliedOutput() * motor.getBusVoltage());
        logDouble(table, "voltageIn", motor.getBusVoltage());
        logBool(table, "hardStopForward", motor.getForwardLimitSwitch().isPressed());
        logBool(table, "hardStopReverse", motor.getReverseLimitSwitch().isPressed());
        logDouble(table, "currentOut", motor.getOutputCurrent());
        logBool(table, "connected", SparkUtils.isConnected(motor));

        logBool(table + "/faults", "can", activeFaults.can);
        logBool(table + "/faults", "escEeprom", activeFaults.escEeprom);
        logBool(table + "/faults", "firmware", activeFaults.firmware);
        logBool(table + "/faults", "gateDriver", activeFaults.gateDriver);
        logBool(table + "/faults", "motorType", activeFaults.motorType);
        logBool(table + "/faults", "other", activeFaults.other);
        logBool(table + "/faults", "sensor", activeFaults.sensor);
        logBool(table + "/faults", "temperature", activeFaults.temperature);
        logBool(table, "criticalFaultsActive", SparkUtils.hasCriticalFaults(activeFaults));

        logBool(table + "/stickyFaults", "can", stickyFaults.can);
        logBool(table + "/stickyFaults", "escEeprom", stickyFaults.escEeprom);
        logBool(table + "/stickyFaults", "firmware", stickyFaults.firmware);
        logBool(table + "/stickyFaults", "gateDriver", stickyFaults.gateDriver);
        logBool(table + "/stickyFaults", "motorType", stickyFaults.motorType);
        logBool(table + "/stickyFaults", "other", stickyFaults.other);
        logBool(table + "/stickyFaults", "sensor", stickyFaults.sensor);
        logBool(table + "/stickyFaults", "temperature", stickyFaults.temperature);
        logBool(table, "criticalStickyFaultsActive", SparkUtils.hasCriticalFaults(stickyFaults));

        logBool(table + "/warnings", "brownout", activeWarnings.brownout);
        logBool(table + "/warnings", "escEeprom", activeWarnings.escEeprom);
        logBool(table + "/warnings", "extEeprom", activeWarnings.extEeprom);
        logBool(table + "/warnings", "hasReset", activeWarnings.hasReset);
        logBool(table + "/warnings", "other", activeWarnings.other);
        logBool(table + "/warnings", "overcurrent", activeWarnings.overcurrent);
        logBool(table + "/warnings", "sensor", activeWarnings.sensor);
        logBool(table + "/warnings", "stall", activeWarnings.stall);
        logBool(table, "criticalWarningsActive", SparkUtils.hasCriticalWarnings(activeWarnings));

        logBool(table + "/stickyWarnings", "brownout", stickyWarnings.brownout);
        logBool(table + "/stickyWarnings", "escEeprom", stickyWarnings.escEeprom);
        logBool(table + "/stickyWarnings", "extEeprom", stickyWarnings.extEeprom);
        logBool(table + "/stickyWarnings", "hasReset", stickyWarnings.hasReset);
        logBool(table + "/stickyWarnings", "other", stickyWarnings.other);
        logBool(table + "/stickyWarnings", "overcurrent", stickyWarnings.overcurrent);
        logBool(table + "/stickyWarnings", "sensor", stickyWarnings.sensor);
        logBool(table + "/stickyWarnings", "stall", stickyWarnings.stall);
        logBool(table, "criticalStickyWarningsActive", SparkUtils.hasCriticalWarnings(stickyWarnings));

        if (additionalData != null && additionalData.isPresent()) {
            for (Entry<String, String> data : additionalData.get().entrySet()) {
                logString(table, data.getKey(), data.getValue());
            }
        }
    }

    public static <T extends SubsystemBase> void logSubsystem(String subsystemName, T subsystem) {
        if (subsystemName == null) {
            reportWarning("Cannot log to an empty subsytemName", true);
            return;
        }
        if (subsystem == null) {
            reportWarning("Cannot log a null subsystem", true);
            return;
        }

        if (subsystem.getCurrentCommand() == null) {
            logString(subsystemName, "currentCommand", "none");
        } else {
            logString(
                    subsystemName,
                    "currentCommand",
                    subsystem.getCurrentCommand().getName());
        }
        if (subsystem.getDefaultCommand() == null) {
            logString(subsystemName, "defaultCommand", "none");
        } else {
            logString(
                    subsystemName,
                    "defaultCommand",
                    subsystem.getDefaultCommand().getName());
        }
    }

    public static void logPDP(PowerDistribution pdp) {
        if (pdp == null) {
            reportWarning("Cannot log a null PDP", true);
            return;
        }

        logDouble("PDP", "batteryVoltage", pdp.getVoltage());
        logDoubleArray("PDP", "currents", pdp.getAllCurrents());
        logDouble("PDP", "totalCurrent", pdp.getTotalCurrent());

        PowerDistributionFaults faults = pdp.getFaults();
        String table = "PDP/faults";
        logBool(table, "channel0Breaker", faults.Channel0BreakerFault);
        logBool(table, "channel1Breaker", faults.Channel1BreakerFault);
        logBool(table, "channel2Breaker", faults.Channel2BreakerFault);
        logBool(table, "channel3Breaker", faults.Channel3BreakerFault);
        logBool(table, "channel4Breaker", faults.Channel4BreakerFault);
        logBool(table, "channel5Breaker", faults.Channel5BreakerFault);
        logBool(table, "channel6Breaker", faults.Channel6BreakerFault);
        logBool(table, "channel7Breaker", faults.Channel7BreakerFault);
        logBool(table, "channel8Breaker", faults.Channel8BreakerFault);
        logBool(table, "channel9Breaker", faults.Channel9BreakerFault);
        logBool(table, "channel10Breaker", faults.Channel10BreakerFault);
        logBool(table, "channel11Breaker", faults.Channel11BreakerFault);
        logBool(table, "channel12Breaker", faults.Channel12BreakerFault);
        logBool(table, "channel13Breaker", faults.Channel13BreakerFault);
        logBool(table, "channel14Breaker", faults.Channel14BreakerFault);
        logBool(table, "channel15Breaker", faults.Channel15BreakerFault);
        logBool(table, "channel16Breaker", faults.Channel16BreakerFault);
        logBool(table, "channel17Breaker", faults.Channel17BreakerFault);
        logBool(table, "channel18Breaker", faults.Channel18BreakerFault);
        logBool(table, "channel19Breaker", faults.Channel19BreakerFault);
        logBool(table, "channel20Breaker", faults.Channel20BreakerFault);
        logBool(table, "channel21Breaker", faults.Channel21BreakerFault);
        logBool(table, "channel22Breaker", faults.Channel22BreakerFault);
        logBool(table, "channel23Breaker", faults.Channel23BreakerFault);
        logBool(table, "brownout", faults.Brownout);
        logBool(table, "canWarning", faults.CanWarning);
        logBool(table, "hardware", faults.HardwareFault);

        PowerDistributionStickyFaults stickyFaults = pdp.getStickyFaults();
        table = "PDP/stickyFaults";
        logBool(table, "channel0Breaker", stickyFaults.Channel0BreakerFault);
        logBool(table, "channel1Breaker", stickyFaults.Channel1BreakerFault);
        logBool(table, "channel2Breaker", stickyFaults.Channel2BreakerFault);
        logBool(table, "channel3Breaker", stickyFaults.Channel3BreakerFault);
        logBool(table, "channel4Breaker", stickyFaults.Channel4BreakerFault);
        logBool(table, "channel5Breaker", stickyFaults.Channel5BreakerFault);
        logBool(table, "channel6Breaker", stickyFaults.Channel6BreakerFault);
        logBool(table, "channel7Breaker", stickyFaults.Channel7BreakerFault);
        logBool(table, "channel8Breaker", stickyFaults.Channel8BreakerFault);
        logBool(table, "channel9Breaker", stickyFaults.Channel9BreakerFault);
        logBool(table, "channel10Breaker", stickyFaults.Channel10BreakerFault);
        logBool(table, "channel11Breaker", stickyFaults.Channel11BreakerFault);
        logBool(table, "channel12Breaker", stickyFaults.Channel12BreakerFault);
        logBool(table, "channel13Breaker", stickyFaults.Channel13BreakerFault);
        logBool(table, "channel14Breaker", stickyFaults.Channel14BreakerFault);
        logBool(table, "channel15Breaker", stickyFaults.Channel15BreakerFault);
        logBool(table, "channel16Breaker", stickyFaults.Channel16BreakerFault);
        logBool(table, "channel17Breaker", stickyFaults.Channel17BreakerFault);
        logBool(table, "channel18Breaker", stickyFaults.Channel18BreakerFault);
        logBool(table, "channel19Breaker", stickyFaults.Channel19BreakerFault);
        logBool(table, "channel20Breaker", stickyFaults.Channel20BreakerFault);
        logBool(table, "channel21Breaker", stickyFaults.Channel21BreakerFault);
        logBool(table, "channel22Breaker", stickyFaults.Channel22BreakerFault);
        logBool(table, "channel23Breaker", stickyFaults.Channel23BreakerFault);
        logBool(table, "brownout", stickyFaults.Brownout);
        logBool(table, "canWarning", stickyFaults.CanWarning);
        logBool(table, "canBusOff", stickyFaults.CanBusOff);
        logBool(table, "hardware", stickyFaults.HardwareFault);
        logBool(table, "firmware", stickyFaults.FirmwareFault);
        logBool(table, "hasReset", stickyFaults.HasReset);
    }

    private static class StackTrace {
        public String location;
        public String trace;
    }

    private static StackTrace generateStackTrace(StackTraceElement[] trace, int offset) {
        // stole this code from DriverStation.class:494
        String locString;
        if (trace.length >= offset + 1) {
            locString = trace[offset].toString();
        } else {
            locString = "";
        }

        StringBuilder traceString = new StringBuilder();
        boolean haveLoc = false;
        for (int i = offset; i < trace.length; i++) {
            String loc = trace[i].toString();
            traceString.append("\tat ").append(loc).append('\n');

            // get first user function
            if (!haveLoc && !loc.startsWith("edu.wpi.first")) {
                locString = loc;
                haveLoc = true;
            }
        }

        StackTrace result = new StackTrace();
        result.location = locString;
        result.trace = traceString.toString();
        return result;
    }
}
