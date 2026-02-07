package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.TurretConstants;

public class Alerts {

    public static final Alert driver1Missing, driver2Missing, fmsConnected, lowBattery, criticalBattery;
    public static final Alert turretDisconnected, turretOverheating, turretWarnings, turretFaults;

    static {
        driver1Missing = new Alert("Driver 1 controller is not plugged in to port 0", AlertType.kWarning);
        driver1Missing.set(false);

        driver2Missing = new Alert("Driver 2 controller is not plugged in to port 1", AlertType.kWarning);
        driver2Missing.set(false);

        fmsConnected = new Alert("FMS connected", AlertType.kInfo);
        fmsConnected.set(false);

        lowBattery = new Alert("Low battery", AlertType.kWarning);
        lowBattery.set(false);

        criticalBattery = new Alert("Very low battery", AlertType.kError);
        criticalBattery.set(false);

        turretDisconnected = new Alert(
                String.format("Missing connection to turret motor (CAN %d)", TurretConstants.turretId),
                AlertType.kError);
        turretDisconnected.set(false);

        turretOverheating = new Alert(
                String.format("Turret motor (CAN %d) is overheating", TurretConstants.turretId), AlertType.kWarning);
        turretOverheating.set(false);

        turretWarnings = new Alert(
                String.format(
                        "Potentially critical warnings are active on turret motor (CAN %d)", TurretConstants.turretId),
                AlertType.kWarning);
        turretWarnings.set(false);

        turretFaults = new Alert(
                String.format(
                        "Potentially critical faults are active on turret motor (CAN %d)", TurretConstants.turretId),
                AlertType.kWarning);
        turretFaults.set(false);
    }

    private Alerts() {}
}
