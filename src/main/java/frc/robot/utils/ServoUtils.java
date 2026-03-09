package frc.robot.utils;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoHub.Faults;
import com.revrobotics.servohub.ServoHub.Warnings;
import com.revrobotics.servohub.ServoHubLowLevel.FirmwareVersion;

public class ServoUtils {

    private ServoUtils() {}

    public static boolean hasCriticalFaults(Faults faults) {
        if (faults == null) return false;

        return faults.firmware || faults.hardware || faults.lowBattery;
    }

    public static boolean hasCriticalWarnings(Warnings warnings) {
        if (warnings == null) return false;

        return warnings.brownout
                || warnings.canBusOff
                || warnings.canWarning
                || warnings.channel0Overcurrent
                || warnings.channel1Overcurrent
                || warnings.channel2Overcurrent
                || warnings.channel3Overcurrent
                || warnings.channel4Overcurrent
                || warnings.channel5Overcurrent
                || warnings.hasReset;
    }

    public static boolean isConnected(ServoHub hub) {
        if (hub == null) {
            Logger.reportWarning("Cannot test connectivity of a null ServoHub", true);
            return false;
        }

        FirmwareVersion version = hub.getFirmwareVersion();
        return version.hardwareMajor() != 0;
    }

    public static double getNormalizedChannelOutput(ServoChannel channel) {
        if (channel == null) {
            Logger.reportWarning("Cannot get the normalized output of a null ServoChannel", true);
            return 0;
        }

        // 500 microseconds is fully CCW
        // 1500 microseconds is center
        // 2500 microseconds is fully CW
        return (channel.getPulseWidth() - 1500) / 1000.0;
    }
}
