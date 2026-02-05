package frc.robot.utils;

import edu.wpi.first.wpilibj.PowerDistribution;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;

public class CANMonitor {

    private static Map<Integer, Boolean> statuses;
    private static Optional<BiConsumer<Integer, Boolean>> connectionChangeCallback;

    static {
        statuses = new HashMap<>();
        connectionChangeCallback = Optional.empty();
    }

    private CANMonitor() {}

    public static void logCANDeviceStatus(String name, int id, boolean connected) {
        if (name == null || name.isEmpty()) name = "Unnamed";

        statuses.put(id, connected);
        Logger.logBool("CAN", String.format("%d_%s", id, name), connected);

        if (connected != statuses.get(id) && connectionChangeCallback != null && connectionChangeCallback.isPresent()) {
            connectionChangeCallback.get().accept(id, connected);
        }
    }

    public static Optional<Boolean> getCANDeviceStatus(int id) {
        if (!statuses.containsKey(id)) return Optional.empty();

        return Optional.of(statuses.get(id));
    }

    /**
     * @param callback Arg 1: The integer ID of the device who's status changed, Arg 2: the new connection status of the device
     */
    public static void setConnectionChangeCallback(BiConsumer<Integer, Boolean> callback) {
        if (callback == null) {
            Logger.reportWarning("Cannot set CAN connection change callback to a null BiConsumer", true);
            return;
        }

        connectionChangeCallback = Optional.of(callback);
    }

    public static void removeConnectionChangeCallback() {
        connectionChangeCallback = Optional.empty();
    }

    public static boolean isPDPConnected(PowerDistribution pdp) {
        if (pdp == null) return false;
        return pdp.getVoltage() != 0;
    }
}
