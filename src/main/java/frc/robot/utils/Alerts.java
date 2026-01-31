package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.IntakeConstants.Roller;

public class Alerts {

    public static final Alert driver1Missing, driver2Missing, fmsConnected, lowBattery, criticalBattery;
    public static final Alert rollerConfigFail, rollerDisconnected, rollerOverheating, rollerFaults, rollerWarnings;

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

        rollerConfigFail = new Alert(
                String.format("Failed to update roller motor (CAN %d) config", Roller.motorID), AlertType.kError);
        rollerConfigFail.set(false);

        rollerDisconnected = new Alert(
                String.format("Missing connection to roller motor (CAN %d)", Roller.motorID), AlertType.kError);
        rollerDisconnected.set(false);

        rollerOverheating =
                new Alert(String.format("Roller motor (CAN %d) is overheating", Roller.motorID), AlertType.kWarning);
        rollerOverheating.set(false);

        rollerFaults = new Alert(
                String.format("Potentially critical faults are active on roller motor (CAN %d)", Roller.motorID),
                AlertType.kWarning);
        rollerFaults.set(false);

        rollerWarnings = new Alert(
                String.format("Potentially critical warnings are active on roller motor (CAN %d)", Roller.motorID),
                AlertType.kWarning);
        rollerWarnings.set(false);
    }

    private Alerts() {}
}
