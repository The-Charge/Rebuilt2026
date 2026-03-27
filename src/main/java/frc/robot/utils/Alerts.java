package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotContainer;
import java.util.Optional;

public class Alerts {

    public static final Alert driver1Missing,
            driver2Missing,
            buttonBoxConnected,
            fmsConnected,
            lowBattery,
            criticalBattery;
    public static Optional<Alert> pdpDisconnected;

    static {
        driver1Missing = new Alert("Driver 1 controller is not plugged in to port 0", AlertType.kWarning);
        driver2Missing = new Alert("Driver 2 controller is not plugged in to port 1", AlertType.kWarning);
        buttonBoxConnected = new Alert("Manual control button box is connected", AlertType.kInfo);

        fmsConnected = new Alert("FMS connected", AlertType.kInfo);

        lowBattery = new Alert("Low battery", AlertType.kWarning);
        criticalBattery = new Alert("Very low battery", AlertType.kError);

        pdpDisconnected = Optional.empty();
    }

    public static void setupDeferredInitializations() {
        pdpDisconnected = Optional.of(
                makeDisconnectAlert("PDP", RobotContainer.getInstance().pdp.getModule() + 1));
    }

    private Alerts() {}

    public static Alert makeConfigFailAlert(String deviceName, int can) {
        return new Alert(String.format("Failed to update %s (CAN %d) config", deviceName, can), AlertType.kError);
    }

    public static Alert makeDisconnectAlert(String deviceName, int can) {
        return new Alert(String.format("Missing connection to %s (CAN %d)", deviceName, can), AlertType.kError);
    }

    public static Alert makeOverheatingAlert(String deviceName, int can) {
        return new Alert(String.format("%s (CAN %d) is overheating", deviceName, can), AlertType.kWarning);
    }

    public static Alert makeCriticalFaultsAlert(String deviceName, int can) {
        return new Alert(
                String.format("Potentially critical faults are active on %s (CAN %d)", deviceName, can),
                AlertType.kWarning);
    }

    public static Alert makeCriticalWarningsAlert(String deviceName, int can) {
        return new Alert(
                String.format("Potentially critical warnings are active on %s (CAN %d)", deviceName, can),
                AlertType.kWarning);
    }
}
