package frc.robot.utils;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Optional;

public class Logger {

    private static enum LoggingLevel {
        ENABLED(true, true),
        NT_ONLY(true, false),
        DISABLED(false, false);

        final boolean logToNT;
        final boolean logToFile;

        LoggingLevel(boolean nt, boolean file) {
            logToNT = nt;
            logToFile = file;
        }
    };

    private static final String PREFIX = "[Logger] ";
    private static final Optional<Time> loopOverrunPeriod = Optional.of(Seconds.of(0.2));
    private static final boolean showJoystickDisconnectWarnings = false;
    private static final boolean ctreLoggingEnabled = false;
    private static final boolean revLoggingEnabled = false;
    private static final LoggingLevel loggingLevel = LoggingLevel.ENABLED;

    private static final Alert notLoggingToFlashdrive;

    private static boolean hasInited;
    private static Optional<NetworkTableInstance> ntInstance;
    private static Optional<NetworkTable> nt;

    static {
        notLoggingToFlashdrive = new Alert(
                "Logger is not logging to a flash drive. Please confirm that the flash drive is securely plugged in and was plugged in before the robot was turned on",
                AlertType.kError);

        hasInited = false;
        ntInstance = Optional.empty();
        nt = Optional.empty();
    }

    private Logger() {}

    public static void init(Robot robot) {
        if (hasInited) return;

        // Adjust loop overrun warning timeout
        if (loopOverrunPeriod.isPresent()) {
            try {
                Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
                watchdogField.setAccessible(true);
                Watchdog watchdog = (Watchdog) watchdogField.get(robot);
                watchdog.setTimeout(loopOverrunPeriod.get().in(Seconds));

                CommandScheduler.getInstance().setPeriod(loopOverrunPeriod.get().in(Seconds));
            } catch (Exception e) {
                Logger.reportWarning("Failed to adjust loop overrun warnings", false);
            }
        }
        DriverStation.silenceJoystickConnectionWarning(!showJoystickDisconnectWarnings);
        SignalLogger.enableAutoLogging(ctreLoggingEnabled);
        if (!revLoggingEnabled) {
            StatusLogger.disableAutoLogging();
        }

        if (loggingLevel.logToNT) {
            ntInstance = Optional.of(NetworkTableInstance.getDefault());
            nt = Optional.of(ntInstance.get().getTable("Logger")); // root table
        }

        if (loggingLevel.logToFile) {
            DataLogManager.start();
            DataLogManager.logConsoleOutput(true);
            DataLogManager.logNetworkTables(true);
            DriverStation.startDataLog(DataLogManager.getLog(), true);

            String logDir = DataLogManager.getLogDir();
            logString("Logger", "loggingDirectory", logDir);

            boolean loggingToFlash =
                    logDir == null ? false : logDir.toLowerCase().startsWith("/u");
            logBool("Logger", "loggingToFlashdrive", loggingToFlash);
            notLoggingToFlashdrive.set(!loggingToFlash);
        }

        println("Logging started");
        hasInited = true;
    }

    public static Optional<NetworkTable> getLoggerTable() {
        return nt;
    }

    /**
     * Use instead of System.out.println
     */
    public static void println(String msg) {
        if (!loggingLevel.logToFile) {
            System.out.println(msg);
            return;
        }

        if (msg == null) {
            msg = "null";
        }
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
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setBoolean(val)) {
            reportWarning(
                    "attempted to log boolean value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logBoolArray(String subsystem, String key, boolean[] val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null boolean[]", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setBooleanArray(val)) {
            reportWarning(
                    "attempted to log boolean[] value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logDouble(String subsystem, String key, double val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setDouble(val)) {
            reportWarning(
                    "attempted to log double value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logDoubleArray(String subsystem, String key, double[] val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null double[]", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setDoubleArray(val)) {
            reportWarning(
                    "attempted to log double[] value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logFloat(String subsystem, String key, float val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setFloat(val)) {
            reportWarning(
                    "attempted to log float value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logFloatArray(String subsystem, String key, float[] val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null float[]", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setFloatArray(val)) {
            reportWarning(
                    "attempted to log float[] value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logLong(String subsystem, String key, long val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setInteger(val)) {
            reportWarning(
                    "attempted to log int value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logLongArray(String subsystem, String key, long[] val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null long[]", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setIntegerArray(val)) {
            reportWarning(
                    "attempted to log int[] value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logRaw(String subsystem, String key, ByteBuffer val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null ByteBuffer", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setRaw(val)) {
            reportWarning(
                    "attempted to log raw value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logRaw(String subsystem, String key, byte[] val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null byte[]", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setRaw(val)) {
            reportWarning(
                    "attempted to log raw value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logString(String subsystem, String key, String val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) val = "";

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setString(val)) {
            reportWarning(
                    "attempted to log String value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logStringArray(String subsystem, String key, String[] val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (key == null || key.isEmpty()) {
            reportWarning("Cannot log to an empty key", true);
            return;
        }
        if (val == null) {
            reportWarning("Cannot log a null String[]", true);
            return;
        }

        String normalized;
        if (subsystem == null || subsystem.isEmpty()) {
            normalized = key;
        } else {
            normalized = subsystem + "/" + key;
        }

        if (!nt.get().getEntry(normalized).setStringArray(val)) {
            reportWarning(
                    "attempted to log String[] value to entry '" + normalized + "' of type "
                            + nt.get().getEntry(normalized).getType().getValueStr(),
                    true);
        }
    }

    public static void logEnum(String subsystem, String key, Enum<?> val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (val == null) {
            reportWarning("Cannot log a null Enum", true);
            return;
        }

        logString(subsystem, key, val.name());
    }

    public static void logEnumArray(String subsystem, String key, Enum<?>[] val) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (val == null) {
            reportWarning("Cannot log a null Enum[]", true);
            return;
        }

        logStringArray(subsystem, key, (String[])
                Arrays.stream(val).map((Enum<?> i) -> i == null ? "" : i.name()).toArray());
    }

    public static void logTalonFXReduced(String subsystem, String name, TalonFX motor) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return;
        }
        if (motor == null) {
            reportWarning("Cannot log a null TalonFX", true);
            return;
        }

        String normalized;
        if (subsystem == null) {
            normalized = name;
        } else {
            normalized = subsystem + "/" + name;
        }

        logDouble(normalized, "tempC", motor.getDeviceTemp().getValue().in(Units.Celsius));
        logDouble(normalized, "voltageIn", motor.getSupplyVoltage().getValue().in(Units.Volts));
        logDouble(normalized, "currentOut", motor.getStatorCurrent().getValue().in(Units.Amps));
        logEnum(normalized, "controlMode", motor.getControlMode().getValue());
    }

    public static void logTalonFX(String subsystem, String name, TalonFX motor) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return;
        }
        if (motor == null) {
            reportWarning("Cannot log a null TalonFX", true);
            return;
        }

        String normalized;
        if (subsystem == null) {
            normalized = name;
        } else {
            normalized = subsystem + "/" + name;
        }

        // TalonFXFaults activeFaults = TalonFXUtils.getAllActiveFaults(motor);
        // TalonFXFaults stickyFaults = TalonFXUtils.getAllStickyFaults(motor);

        logDouble(normalized, "positionRots", motor.getPosition().getValue().in(Units.Rotations));
        logDouble(normalized, "velocityRPM", motor.getVelocity().getValue().in(Units.RPM));
        logDouble(normalized, "tempC", motor.getDeviceTemp().getValue().in(Units.Celsius));
        logDouble(normalized, "dutyCycle", motor.get());
        logDouble(normalized, "voltageOut", motor.getMotorVoltage().getValue().in(Units.Volts));
        logDouble(normalized, "voltageIn", motor.getSupplyVoltage().getValue().in(Units.Volts));
        // logBool(normalized, "hardStopForward", activeFaults.forwardHardLimit());
        // logBool(normalized, "hardStopReverse", activeFaults.reverseHardLimit());
        // logBool(normalized, "softStopForward", activeFaults.forwardSoftLimit());
        // logBool(normalized, "softStopReverse", activeFaults.reverseSoftLimit());
        logBool(normalized, "hardStopForward", motor.getFault_ForwardHardLimit().getValue());
        logBool(normalized, "hardStopReverse", motor.getFault_ReverseHardLimit().getValue());
        logBool(normalized, "softStopForward", motor.getFault_ForwardSoftLimit().getValue());
        logBool(normalized, "softStopReverse", motor.getFault_ReverseSoftLimit().getValue());
        logDouble(normalized, "currentOut", motor.getStatorCurrent().getValue().in(Units.Amps));
        // logDouble(normalized, "currentIn", motor.getSupplyCurrent().getValue().in(Units.Amps));
        logEnum(normalized, "controlMode", motor.getControlMode().getValue());
        logDouble(
                normalized,
                "closedLoopRef",
                motor.getClosedLoopReference().getValue().doubleValue());
        logBool(normalized, "enabled", motor.getDeviceEnable().getValue() == DeviceEnableValue.Enabled);
        // logBool(normalized, "connected", motor.isConnected());
        // logBool(normalized, "alive", motor.isAlive());

        // logBool(normalized + "/faults", "bootDuringEnable", activeFaults.bootDuringEnable());
        // logBool(normalized + "/faults", "bridgeBrownout", activeFaults.bridgeBrownout());
        // logBool(normalized + "/faults", "deviceTemp", activeFaults.deviceTemp());
        // logBool(normalized + "/faults", "forwardHardLimit", activeFaults.forwardHardLimit());
        // logBool(normalized + "/faults", "fowardSoftLimit", activeFaults.forwardSoftLimit());
        // logBool(normalized + "/faults", "fusedSensorOutOfSync", activeFaults.fusedSensorOutOfSync());
        // logBool(normalized + "/faults", "hardware", activeFaults.hardware());
        // logBool(normalized + "/faults", "missingDifferentialFX", activeFaults.missingDifferentialFX());
        // logBool(normalized + "/faults", "missingHardLimitRemote", activeFaults.missingHardLimitRemote());
        // logBool(normalized + "/faults", "missingSoftLimitRemote", activeFaults.missingSoftLimitRemote());
        // logBool(normalized + "/faults", "overSupplyV", activeFaults.overSupplyV());
        // logBool(normalized + "/faults", "procTemp", activeFaults.procTemp());
        // logBool(normalized + "/faults", "remoteSensorDataInvalid", activeFaults.remoteSensorDataInvalid());
        // logBool(normalized + "/faults", "remoteSensorPosOverflow", activeFaults.remoteSensorPosOverflow());
        // logBool(normalized + "/faults", "remoteSensorReset", activeFaults.remoteSensorReset());
        // logBool(normalized + "/faults", "reverseHardLimit", activeFaults.reverseHardLimit());
        // logBool(normalized + "/faults", "reverseSoftLimit", activeFaults.reverseSoftLimit());
        // logBool(normalized + "/faults", "staticBrakeDisabled", activeFaults.staticBrakeDisabled());
        // logBool(normalized + "/faults", "statorCurrLimit", activeFaults.statorCurrLimit());
        // logBool(normalized + "/faults", "supplyCurrLimit", activeFaults.supplyCurrLimit());
        // logBool(normalized + "/faults", "undervoltage", activeFaults.undervoltage());
        // logBool(normalized + "/faults", "unlicensedFeatureInUse", activeFaults.unlicensedFeatureInUse());
        // logBool(normalized + "/faults", "unstableSupplyV", activeFaults.unstableSupplyV());
        // logBool(
        //         normalized + "/faults",
        //         "usingFusedCANCoderWhileUnlicensed",
        //         activeFaults.usingFusedCANCoderWhileUnlicensed());
        // logBool(normalized, "criticalFaultsActive", activeFaults.hasCriticalFaults());

        // logBool(normalized + "/stickyFaults", "bootDuringEnable", stickyFaults.bootDuringEnable());
        // logBool(normalized + "/stickyFaults", "bridgeBrownout", stickyFaults.bridgeBrownout());
        // logBool(normalized + "/stickyFaults", "deviceTemp", stickyFaults.deviceTemp());
        // logBool(normalized + "/stickyFaults", "forwardHardLimit", stickyFaults.forwardHardLimit());
        // logBool(normalized + "/stickyFaults", "fowardSoftLimit", stickyFaults.forwardSoftLimit());
        // logBool(normalized + "/stickyFaults", "fusedSensorOutOfSync", stickyFaults.fusedSensorOutOfSync());
        // logBool(normalized + "/stickyFaults", "hardware", stickyFaults.hardware());
        // logBool(normalized + "/stickyFaults", "missingDifferentialFX", stickyFaults.missingDifferentialFX());
        // logBool(normalized + "/stickyFaults", "missingHardLimitRemote", stickyFaults.missingHardLimitRemote());
        // logBool(normalized + "/stickyFaults", "missingSoftLimitRemote", stickyFaults.missingSoftLimitRemote());
        // logBool(normalized + "/stickyFaults", "overSupplyV", stickyFaults.overSupplyV());
        // logBool(normalized + "/stickyFaults", "procTemp", stickyFaults.procTemp());
        // logBool(normalized + "/stickyFaults", "remoteSensorDataInvalid", stickyFaults.remoteSensorDataInvalid());
        // logBool(normalized + "/stickyFaults", "remoteSensorPosOverflow", stickyFaults.remoteSensorPosOverflow());
        // logBool(normalized + "/stickyFaults", "remoteSensorReset", stickyFaults.remoteSensorReset());
        // logBool(normalized + "/stickyFaults", "reverseHardLimit", stickyFaults.reverseHardLimit());
        // logBool(normalized + "/stickyFaults", "reverseSoftLimit", stickyFaults.reverseSoftLimit());
        // logBool(normalized + "/stickyFaults", "staticBrakeDisabled", stickyFaults.staticBrakeDisabled());
        // logBool(normalized + "/stickyFaults", "statorCurrLimit", stickyFaults.statorCurrLimit());
        // logBool(normalized + "/stickyFaults", "supplyCurrLimit", stickyFaults.supplyCurrLimit());
        // logBool(normalized + "/stickyFaults", "undervoltage", stickyFaults.undervoltage());
        // logBool(normalized + "/stickyFaults", "unlicensedFeatureInUse", stickyFaults.unlicensedFeatureInUse());
        // logBool(normalized + "/stickyFaults", "unstableSupplyV", stickyFaults.unstableSupplyV());
        // logBool(
        //         normalized + "/stickyFaults",
        //         "usingFusedCANCoderWhileUnlicensed",
        //         stickyFaults.usingFusedCANCoderWhileUnlicensed());
        // logBool(normalized, "criticalStickyFaultsActive", stickyFaults.hasCriticalFaults());
    }

    public static void logSparkMotor(String subsystem, String name, SparkBase motor) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return;
        }
        if (motor == null) {
            reportWarning("Cannot log a null SparkMax", true);
            return;
        }

        String normalized;
        if (subsystem == null) {
            normalized = name;
        } else {
            normalized = subsystem + "/" + name;
        }

        // Faults activeFaults = motor.getFaults();
        // Faults stickyFaults = motor.getStickyFaults();
        // Warnings activeWarnings = motor.getWarnings();
        // Warnings stickyWarnings = motor.getStickyWarnings();

        logDouble(normalized, "positionRots", motor.getEncoder().getPosition());
        logDouble(normalized, "velocityRPM", motor.getEncoder().getVelocity());
        logDouble(normalized, "tempC", motor.getMotorTemperature());
        logDouble(normalized, "dutyCycle", motor.getAppliedOutput());
        logDouble(normalized, "voltageOut", motor.getAppliedOutput() * motor.getBusVoltage());
        logDouble(normalized, "voltageIn", motor.getBusVoltage());
        logBool(normalized, "hardStopForward", motor.getForwardLimitSwitch().isPressed());
        logBool(normalized, "hardStopReverse", motor.getReverseLimitSwitch().isPressed());
        logBool(normalized, "softStopForward", motor.getForwardSoftLimit().isReached());
        logBool(normalized, "softStopReverse", motor.getReverseSoftLimit().isReached());
        logDouble(normalized, "currentOut", motor.getOutputCurrent());
        logString(
                normalized,
                "closedLoopControlType",
                motor.getClosedLoopController().getControlType().toString());
        logDouble(normalized, "iAccum", motor.getClosedLoopController().getIAccum());
        // logBool(normalized, "connected", SparkUtils.isConnected(motor));

        // logBool(normalized + "/faults", "can", activeFaults.can);
        // logBool(normalized + "/faults", "escEeprom", activeFaults.escEeprom);
        // logBool(normalized + "/faults", "firmware", activeFaults.firmware);
        // logBool(normalized + "/faults", "gateDriver", activeFaults.gateDriver);
        // logBool(normalized + "/faults", "motorType", activeFaults.motorType);
        // logBool(normalized + "/faults", "other", activeFaults.other);
        // logBool(normalized + "/faults", "sensor", activeFaults.sensor);
        // logBool(normalized + "/faults", "temperature", activeFaults.temperature);
        // logBool(normalized, "criticalFaultsActive", SparkUtils.hasCriticalFaults(activeFaults));

        // logBool(normalized + "/stickyFaults", "can", stickyFaults.can);
        // logBool(normalized + "/stickyFaults", "escEeprom", stickyFaults.escEeprom);
        // logBool(normalized + "/stickyFaults", "firmware", stickyFaults.firmware);
        // logBool(normalized + "/stickyFaults", "gateDriver", stickyFaults.gateDriver);
        // logBool(normalized + "/stickyFaults", "motorType", stickyFaults.motorType);
        // logBool(normalized + "/stickyFaults", "other", stickyFaults.other);
        // logBool(normalized + "/stickyFaults", "sensor", stickyFaults.sensor);
        // logBool(normalized + "/stickyFaults", "temperature", stickyFaults.temperature);
        // logBool(normalized, "criticalStickyFaultsActive", SparkUtils.hasCriticalFaults(stickyFaults));

        // logBool(normalized + "/warnings", "brownout", activeWarnings.brownout);
        // logBool(normalized + "/warnings", "escEeprom", activeWarnings.escEeprom);
        // logBool(normalized + "/warnings", "extEeprom", activeWarnings.extEeprom);
        // logBool(normalized + "/warnings", "hasReset", activeWarnings.hasReset);
        // logBool(normalized + "/warnings", "other", activeWarnings.other);
        // logBool(normalized + "/warnings", "overcurrent", activeWarnings.overcurrent);
        // logBool(normalized + "/warnings", "sensor", activeWarnings.sensor);
        // logBool(normalized + "/warnings", "stall", activeWarnings.stall);
        // logBool(normalized, "criticalWarningsActive", SparkUtils.hasCriticalWarnings(activeWarnings));

        // logBool(normalized + "/stickyWarnings", "brownout", stickyWarnings.brownout);
        // logBool(normalized + "/stickyWarnings", "escEeprom", stickyWarnings.escEeprom);
        // logBool(normalized + "/stickyWarnings", "extEeprom", stickyWarnings.extEeprom);
        // logBool(normalized + "/stickyWarnings", "hasReset", stickyWarnings.hasReset);
        // logBool(normalized + "/stickyWarnings", "other", stickyWarnings.other);
        // logBool(normalized + "/stickyWarnings", "overcurrent", stickyWarnings.overcurrent);
        // logBool(normalized + "/stickyWarnings", "sensor", stickyWarnings.sensor);
        // logBool(normalized + "/stickyWarnings", "stall", stickyWarnings.stall);
        // logBool(normalized, "criticalStickyWarningsActive", SparkUtils.hasCriticalWarnings(stickyWarnings));
    }

    public static <T extends Subsystem> void logSubsystem(String subsystemName, T subsystem) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (subsystemName == null || subsystemName.isEmpty()) {
            reportWarning("Cannot log to an empty subsytemName", true);
            return;
        }
        if (subsystem == null) {
            reportWarning("Cannot log a null subsystem", true);
            return;
        }

        Command currentCommand = subsystem.getCurrentCommand();
        if (currentCommand == null) {
            logString(subsystemName, "currentCommand", "none");
        } else {
            logString(subsystemName, "currentCommand", currentCommand.getName());
        }

        Command defaultCommand = subsystem.getDefaultCommand();
        if (defaultCommand == null) {
            logString(subsystemName, "defaultCommand", "none");
        } else {
            logString(subsystemName, "defaultCommand", defaultCommand.getName());
        }
    }

    public static void logPDP(PowerDistribution pdp) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (pdp == null) {
            reportWarning("Cannot log a null PDP", true);
            return;
        }

        logDouble("PDP", "batteryVoltage", pdp.getVoltage());
        logDoubleArray("PDP", "currents", pdp.getAllCurrents());
        logDouble("PDP", "totalCurrent", pdp.getTotalCurrent());
        // logBool("PDP", "connected", MiscUtils.isPDPConnected(pdp));

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
        // logBool(table, "brownout", faults.Brownout);
        // logBool(table, "canWarning", faults.CanWarning);
        // logBool(table, "hardware", faults.HardwareFault);

        // PowerDistributionStickyFaults stickyFaults = pdp.getStickyFaults();
        // table = "PDP/stickyFaults";
        // logBool(table, "channel0Breaker", stickyFaults.Channel0BreakerFault);
        // logBool(table, "channel1Breaker", stickyFaults.Channel1BreakerFault);
        // logBool(table, "channel2Breaker", stickyFaults.Channel2BreakerFault);
        // logBool(table, "channel3Breaker", stickyFaults.Channel3BreakerFault);
        // logBool(table, "channel4Breaker", stickyFaults.Channel4BreakerFault);
        // logBool(table, "channel5Breaker", stickyFaults.Channel5BreakerFault);
        // logBool(table, "channel6Breaker", stickyFaults.Channel6BreakerFault);
        // logBool(table, "channel7Breaker", stickyFaults.Channel7BreakerFault);
        // logBool(table, "channel8Breaker", stickyFaults.Channel8BreakerFault);
        // logBool(table, "channel9Breaker", stickyFaults.Channel9BreakerFault);
        // logBool(table, "channel10Breaker", stickyFaults.Channel10BreakerFault);
        // logBool(table, "channel11Breaker", stickyFaults.Channel11BreakerFault);
        // logBool(table, "channel12Breaker", stickyFaults.Channel12BreakerFault);
        // logBool(table, "channel13Breaker", stickyFaults.Channel13BreakerFault);
        // logBool(table, "channel14Breaker", stickyFaults.Channel14BreakerFault);
        // logBool(table, "channel15Breaker", stickyFaults.Channel15BreakerFault);
        // logBool(table, "channel16Breaker", stickyFaults.Channel16BreakerFault);
        // logBool(table, "channel17Breaker", stickyFaults.Channel17BreakerFault);
        // logBool(table, "channel18Breaker", stickyFaults.Channel18BreakerFault);
        // logBool(table, "channel19Breaker", stickyFaults.Channel19BreakerFault);
        // logBool(table, "channel20Breaker", stickyFaults.Channel20BreakerFault);
        // logBool(table, "channel21Breaker", stickyFaults.Channel21BreakerFault);
        // logBool(table, "channel22Breaker", stickyFaults.Channel22BreakerFault);
        // logBool(table, "channel23Breaker", stickyFaults.Channel23BreakerFault);
        // logBool(table, "brownout", stickyFaults.Brownout);
        // logBool(table, "canWarning", stickyFaults.CanWarning);
        // logBool(table, "canBusOff", stickyFaults.CanBusOff);
        // logBool(table, "hardware", stickyFaults.HardwareFault);
        // logBool(table, "firmware", stickyFaults.FirmwareFault);
        // logBool(table, "hasReset", stickyFaults.HasReset);
    }

    public static void logServo(String subsystem, String name, Servo servo) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return;

        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return;
        }
        if (servo == null) {
            reportWarning("Cannot log a null Servo", true);
            return;
        }

        String normalized;
        if (subsystem == null) {
            normalized = name;
        } else {
            normalized = subsystem + "/" + name;
        }

        logDouble(normalized, "position", servo.get());
    }

    public static <T extends StructSerializable, S extends Struct<T>> Optional<StructPublisher<T>> makeStructPublisher(
            String subsystem, String name, S struct) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return Optional.empty();

        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return Optional.empty();
        }

        if (getLoggerTable().isEmpty()) return Optional.empty();

        String normalized;
        if (subsystem == null) {
            normalized = name;
        } else {
            normalized = subsystem + "/" + name;
        }

        return Optional.of(
                getLoggerTable().get().getStructTopic(normalized, struct).publish());
    }

    public static <T extends StructSerializable, S extends Struct<T>>
            Optional<StructArrayPublisher<T>> makeStructArrayPublisher(String subsystem, String name, S struct) {
        if (!loggingLevel.logToNT || nt.isEmpty()) return Optional.empty();

        if (name == null || name.isEmpty()) {
            reportWarning("Cannot log under an empty name", true);
            return Optional.empty();
        }

        if (getLoggerTable().isEmpty()) return Optional.empty();

        String normalized;
        if (subsystem == null) {
            normalized = name;
        } else {
            normalized = subsystem + "/" + name;
        }

        return Optional.of(
                getLoggerTable().get().getStructArrayTopic(normalized, struct).publish());
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
