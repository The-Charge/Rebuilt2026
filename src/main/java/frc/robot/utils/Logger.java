package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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

    public static void log(String subsystem, String key, boolean val) {
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

    public static void log(String subsystem, String key, boolean[] val) {
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

    public static void log(String subsystem, String key, double val) {
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

    public static void log(String subsystem, String key, double[] val) {
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

    public static void log(String subsystem, String key, float val) {
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

    public static void log(String subsystem, String key, float[] val) {
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

    public static void log(String subsystem, String key, long val) {
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

    public static void log(String subsystem, String key, long[] val) {
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

    public static void log(String subsystem, String key, ByteBuffer val) {
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

    public static void log(String subsystem, String key, byte[] val) {
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

    public static void log(String subsystem, String key, String val) {
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

    public static void log(String subsystem, String key, String[] val) {
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

    public static void log(String subsystem, String key, Enum<?> val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null Enum", true);
            return;
        }

        log(subsystem, key, val.name());
    }

    public static void log(String subsystem, String key, Enum<?>[] val) {
        if (subsystem == null) subsystem = "";
        if (key == null) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null Enum[]", true);
            return;
        }

        log(subsystem, key, (String[])
                Arrays.stream(val).map((Enum<?> i) -> i == null ? "" : i.name()).toArray());
    }

    public static void log(String subsystem, String name, TalonFX motor, Optional<Map<String, String>> additionalData) {
        if (subsystem == null) subsystem = "";
        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return;
        }
        if (motor == null) {
            reportWarning("Cannot log a null TalonFX", true);
            return;
        }

        String root = subsystem + "/" + name;

        TalonFXFaults activeFaults = TalonFXUtils.getAllActiveFaults(motor);
        TalonFXFaults stickyFaults = TalonFXUtils.getAllStickyFaults(motor);

        log(root, "positionRots", motor.getPosition().getValue().abs(Units.Rotations));
        log(root, "velocityRPM", motor.getVelocity().getValue().abs(Units.RPM));
        log(root, "tempC", motor.getDeviceTemp().getValue().abs(Units.Celsius));
        log(root, "dutyCycle", motor.get());
        log(root, "voltageOut", motor.getMotorVoltage().getValue().abs(Units.Volts));
        log(root, "voltageIn", motor.getSupplyVoltage().getValue().abs(Units.Volts));
        log(root, "hardStopForward", activeFaults.forwardHardLimit());
        log(root, "hardStopReverse", activeFaults.reverseHardLimit());
        log(root, "softStopForward", activeFaults.forwardSoftLimit());
        log(root, "softStopReverse", activeFaults.reverseSoftLimit());
        log(root, "currentOut", motor.getStatorCurrent().getValue().abs(Units.Amps));
        log(root, "currentIn", motor.getSupplyCurrent().getValue().abs(Units.Amps));
        log(root, "controlMode", motor.getControlMode().getValue());
        log(root, "targetRots", motor.getClosedLoopReference().getValue().doubleValue());
        log(root, "enabled", motor.getDeviceEnable().getValue() == DeviceEnableValue.Enabled);
        log(root, "connected", motor.isConnected());
        log(root, "alive", motor.isAlive());

        log(root + "/faults", "bootDuringEnable", activeFaults.bootDuringEnable());
        log(root + "/faults", "bridgeBrownout", activeFaults.bridgeBrownout());
        log(root + "/faults", "deviceTemp", activeFaults.deviceTemp());
        log(root + "/faults", "forwardHardLimit", activeFaults.forwardHardLimit());
        log(root + "/faults", "fowardSoftLimit", activeFaults.forwardSoftLimit());
        log(root + "/faults", "fusedSensorOutOfSync", activeFaults.fusedSensorOutOfSync());
        log(root + "/faults", "hardware", activeFaults.hardware());
        log(root + "/faults", "missingDifferentialFX", activeFaults.missingDifferentialFX());
        log(root + "/faults", "missingHardLimitRemote", activeFaults.missingHardLimitRemote());
        log(root + "/faults", "missingSoftLimitRemote", activeFaults.missingSoftLimitRemote());
        log(root + "/faults", "overSupplyV", activeFaults.overSupplyV());
        log(root + "/faults", "procTemp", activeFaults.procTemp());
        log(root + "/faults", "remoteSensorDataInvalid", activeFaults.remoteSensorDataInvalid());
        log(root + "/faults", "remoteSensorPosOverflow", activeFaults.remoteSensorPosOverflow());
        log(root + "/faults", "remoteSensorReset", activeFaults.remoteSensorReset());
        log(root + "/faults", "reverseHardLimit", activeFaults.reverseHardLimit());
        log(root + "/faults", "reverseSoftLimit", activeFaults.reverseSoftLimit());
        log(root + "/faults", "staticBrakeDisabled", activeFaults.staticBrakeDisabled());
        log(root + "/faults", "statorCurrLimit", activeFaults.statorCurrLimit());
        log(root + "/faults", "supplyCurrLimit", activeFaults.supplyCurrLimit());
        log(root + "/faults", "undervoltage", activeFaults.undervoltage());
        log(root + "/faults", "unlicensedFeatureInUse", activeFaults.unlicensedFeatureInUse());
        log(root + "/faults", "unstableSupplyV", activeFaults.unstableSupplyV());
        log(root + "/faults", "usingFusedCANCoderWhileUnlicensed", activeFaults.usingFusedCANCoderWhileUnlicensed());
        log(root, "criticalFaultsActive", activeFaults.hasCriticalFaults());

        log(root + "/stickyFaults", "bootDuringEnable", stickyFaults.bootDuringEnable());
        log(root + "/stickyFaults", "bridgeBrownout", stickyFaults.bridgeBrownout());
        log(root + "/stickyFaults", "deviceTemp", stickyFaults.deviceTemp());
        log(root + "/stickyFaults", "forwardHardLimit", stickyFaults.forwardHardLimit());
        log(root + "/stickyFaults", "fowardSoftLimit", stickyFaults.forwardSoftLimit());
        log(root + "/stickyFaults", "fusedSensorOutOfSync", stickyFaults.fusedSensorOutOfSync());
        log(root + "/stickyFaults", "hardware", stickyFaults.hardware());
        log(root + "/stickyFaults", "missingDifferentialFX", stickyFaults.missingDifferentialFX());
        log(root + "/stickyFaults", "missingHardLimitRemote", stickyFaults.missingHardLimitRemote());
        log(root + "/stickyFaults", "missingSoftLimitRemote", stickyFaults.missingSoftLimitRemote());
        log(root + "/stickyFaults", "overSupplyV", stickyFaults.overSupplyV());
        log(root + "/stickyFaults", "procTemp", stickyFaults.procTemp());
        log(root + "/stickyFaults", "remoteSensorDataInvalid", stickyFaults.remoteSensorDataInvalid());
        log(root + "/stickyFaults", "remoteSensorPosOverflow", stickyFaults.remoteSensorPosOverflow());
        log(root + "/stickyFaults", "remoteSensorReset", stickyFaults.remoteSensorReset());
        log(root + "/stickyFaults", "reverseHardLimit", stickyFaults.reverseHardLimit());
        log(root + "/stickyFaults", "reverseSoftLimit", stickyFaults.reverseSoftLimit());
        log(root + "/stickyFaults", "staticBrakeDisabled", stickyFaults.staticBrakeDisabled());
        log(root + "/stickyFaults", "statorCurrLimit", stickyFaults.statorCurrLimit());
        log(root + "/stickyFaults", "supplyCurrLimit", stickyFaults.supplyCurrLimit());
        log(root + "/stickyFaults", "undervoltage", stickyFaults.undervoltage());
        log(root + "/stickyFaults", "unlicensedFeatureInUse", stickyFaults.unlicensedFeatureInUse());
        log(root + "/stickyFaults", "unstableSupplyV", stickyFaults.unstableSupplyV());
        log(
                root + "/stickyFaults",
                "usingFusedCANCoderWhileUnlicensed",
                stickyFaults.usingFusedCANCoderWhileUnlicensed());
        log(root, "criticalStickyFaultsActive", stickyFaults.hasCriticalFaults());

        if (additionalData != null && additionalData.isPresent()) {
            for (Entry<String, String> data : additionalData.get().entrySet()) {
                log(root, data.getKey(), data.getValue());
            }
        }
    }

    public static void log(
            String subsystem, String name, SparkMax motor, Optional<Map<String, String>> additionalData) {
        if (subsystem == null) subsystem = "";
        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return;
        }
        if (motor == null) {
            reportWarning("Cannot log a null SparkMax", true);
            return;
        }

        String root = subsystem + "/" + name;

        Faults activeFaults = motor.getFaults();
        Faults stickyFaults = motor.getStickyFaults();
        Warnings activeWarnings = motor.getWarnings();
        Warnings stickyWarnings = motor.getStickyWarnings();

        log(root, "positionRots", motor.getEncoder().getPosition());
        log(root, "velocityRPM", motor.getEncoder().getVelocity());
        log(root, "tempC", motor.getMotorTemperature());
        log(root, "dutyCycle", motor.getAppliedOutput());
        log(root, "voltageOut", motor.getAppliedOutput() * motor.getBusVoltage());
        log(root, "voltageIn", motor.getBusVoltage());
        log(root, "hardStopForward", motor.getForwardLimitSwitch().isPressed());
        log(root, "hardStopReverse", motor.getReverseLimitSwitch().isPressed());
        log(root, "currentOut", motor.getOutputCurrent());
        log(root, "connected", SparkUtils.isConnected(motor));

        log(root + "/faults", "can", activeFaults.can);
        log(root + "/faults", "escEeprom", activeFaults.escEeprom);
        log(root + "/faults", "firmware", activeFaults.firmware);
        log(root + "/faults", "gateDriver", activeFaults.gateDriver);
        log(root + "/faults", "motorType", activeFaults.motorType);
        log(root + "/faults", "other", activeFaults.other);
        log(root + "/faults", "sensor", activeFaults.sensor);
        log(root + "/faults", "temperature", activeFaults.temperature);
        log(root, "criticalFaultsActive", SparkUtils.hasCriticalFaults(activeFaults));

        log(root + "/stickyFaults", "can", stickyFaults.can);
        log(root + "/stickyFaults", "escEeprom", stickyFaults.escEeprom);
        log(root + "/stickyFaults", "firmware", stickyFaults.firmware);
        log(root + "/stickyFaults", "gateDriver", stickyFaults.gateDriver);
        log(root + "/stickyFaults", "motorType", stickyFaults.motorType);
        log(root + "/stickyFaults", "other", stickyFaults.other);
        log(root + "/stickyFaults", "sensor", stickyFaults.sensor);
        log(root + "/stickyFaults", "temperature", stickyFaults.temperature);
        log(root, "criticalStickyFaultsActive", SparkUtils.hasCriticalFaults(stickyFaults));

        log(root + "/warnings", "brownout", activeWarnings.brownout);
        log(root + "/warnings", "escEeprom", activeWarnings.escEeprom);
        log(root + "/warnings", "extEeprom", activeWarnings.extEeprom);
        log(root + "/warnings", "hasReset", activeWarnings.hasReset);
        log(root + "/warnings", "other", activeWarnings.other);
        log(root + "/warnings", "overcurrent", activeWarnings.overcurrent);
        log(root + "/warnings", "sensor", activeWarnings.sensor);
        log(root + "/warnings", "stall", activeWarnings.stall);
        log(root, "criticalWarningsActive", SparkUtils.hasCriticalWarnings(activeWarnings));

        log(root + "/stickyWarnings", "brownout", stickyWarnings.brownout);
        log(root + "/stickyWarnings", "escEeprom", stickyWarnings.escEeprom);
        log(root + "/stickyWarnings", "extEeprom", stickyWarnings.extEeprom);
        log(root + "/stickyWarnings", "hasReset", stickyWarnings.hasReset);
        log(root + "/stickyWarnings", "other", stickyWarnings.other);
        log(root + "/stickyWarnings", "overcurrent", stickyWarnings.overcurrent);
        log(root + "/stickyWarnings", "sensor", stickyWarnings.sensor);
        log(root + "/stickyWarnings", "stall", stickyWarnings.stall);
        log(root, "criticalStickyWarningsActive", SparkUtils.hasCriticalWarnings(stickyWarnings));

        if (additionalData != null && additionalData.isPresent()) {
            for (Entry<String, String> data : additionalData.get().entrySet()) {
                log(root, data.getKey(), data.getValue());
            }
        }
    }

    public static <T extends SubsystemBase> void log(String subsystemName, T subsystem) {
        if (subsystemName == null) {
            reportWarning("Cannot log to an empty subsytemName", true);
            return;
        }
        if (subsystem == null) {
            reportWarning("Cannot log a null subsystem", true);
            return;
        }

        if (subsystem.getCurrentCommand() == null) {
            log(subsystemName, "currentCommand", "none");
        } else {
            log(subsystemName, "currentCommand", subsystem.getCurrentCommand().getName());
        }
        if (subsystem.getDefaultCommand() == null) {
            log(subsystemName, "defaultCommand", "none");
        } else {
            log(subsystemName, "defaultCommand", subsystem.getDefaultCommand().getName());
        }
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
