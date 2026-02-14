package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.HoodConfig;
import frc.robot.constants.TurretConstants;

public class Alerts {

    public static final Alert driver1Missing, driver2Missing, fmsConnected, lowBattery, criticalBattery;
    public static final Alert turretDisconnected, turretOverheating, turretWarnings, turretFaults;
    public static final Alert shooterDisconnected, shooterOverheating, shooterWarnings, shooterFaults;
    public static final Alert hoodDisconnected, hoodOverheating, hoodWarnings, hoodFaults;

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
                String.format("Missing connection to turret motor (CAN %d)", TurretConstants.motorID),
                AlertType.kError);
        turretDisconnected.set(false);

        turretOverheating = new Alert(
                String.format("Turret motor (CAN %d) is overheating", TurretConstants.motorID), AlertType.kWarning);
        turretOverheating.set(false);

        turretWarnings = new Alert(
                String.format(
                        "Potentially critical warnings are active on turret motor (CAN %d)", TurretConstants.motorID),
                AlertType.kWarning);
        turretWarnings.set(false);

        turretFaults = new Alert(
                String.format(
                        "Potentially critical faults are active on turret motor (CAN %d)", TurretConstants.motorID),
                AlertType.kWarning);
        turretFaults.set(false);

        shooterDisconnected = new Alert(
                String.format("Missing connection to shooter motor (CAN %d)", ShooterConstants.ShootConfig.ID),
                AlertType.kError);
        shooterDisconnected.set(false);

        shooterOverheating = new Alert(
                String.format("Shooter motor (CAN %d) is overheating", ShooterConstants.ShootConfig.ID),
                AlertType.kWarning);
        shooterOverheating.set(false);

        shooterWarnings = new Alert(
                String.format(
                        "Potentially critical warnings are active on shooter motor (CAN %d)",
                        ShooterConstants.ShootConfig.ID),
                AlertType.kWarning);
        shooterWarnings.set(false);

        shooterFaults = new Alert(
                String.format(
                        "Potentially critical faults are active on shooter motor (CAN %d)",
                        ShooterConstants.ShootConfig.ID),
                AlertType.kWarning);
        shooterFaults.set(false);

        hoodDisconnected =
                new Alert(String.format("Missing connection to hood motor (CAN %d)", HoodConfig.ID), AlertType.kError);
        hoodDisconnected.set(false);

        hoodOverheating =
                new Alert(String.format("Hood motor (CAN %d) is overheating", HoodConfig.ID), AlertType.kWarning);
        hoodOverheating.set(false);

        hoodWarnings = new Alert(
                String.format("Potentially critical warnings are active on hood motor (CAN %d)", HoodConfig.ID),
                AlertType.kWarning);
        hoodWarnings.set(false);

        hoodFaults = new Alert(
                String.format("Potentially critical faults are active on hood motor (CAN %d)", HoodConfig.ID),
                AlertType.kWarning);
        hoodFaults.set(false);
    }

    private Alerts() {}
}
