package frc.robot.utils;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkMax;

public class SparkMaxUtils {

    private SparkMaxUtils() {}

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

    public static boolean isConnected(SparkMax motor) {
        if (motor == null) {
            Logger.reportWarning("Cannot test connectivity of a null motor", true);
            return false;
        }

        return motor.getFirmwareVersion() != 0;
    }
}
