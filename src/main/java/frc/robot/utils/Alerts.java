package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.ClimberConstants;

public class Alerts {

    public static final Alert driver1Missing, driver2Missing, fmsConnected, lowBattery, criticalBattery;
    public static final Alert climberDisconnected, climberOverheating, climberFaults, climberConfigFail;

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

        climberDisconnected = new Alert(
                String.format("Missing connection to climber motor (CAN %d)", ClimberConstants.motorID),
                AlertType.kError);
        climberDisconnected.set(false);

        climberOverheating = new Alert(
                String.format("Climber motor (CAN %d) is overheating", ClimberConstants.motorID), AlertType.kWarning);
        climberOverheating.set(false);

        climberFaults = new Alert(
                String.format(
                        "Potentially critical faults are active on climber motor (CAN %d)", ClimberConstants.motorID),
                AlertType.kWarning);
        climberFaults.set(false);

        climberConfigFail = new Alert(
                String.format("Failed to update climber motor (CAN %d) config", ClimberConstants.motorID),
                AlertType.kError);
        climberConfigFail.set(false);
    }

    private Alerts() {}
}
