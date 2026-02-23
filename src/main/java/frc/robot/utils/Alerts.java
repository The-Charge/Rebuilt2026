package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants.Roller;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;

public class Alerts {

    // general alerts
    public static final Alert driver1Missing,
            driver2Missing,
            fmsConnected,
            lowBattery,
            criticalBattery,
            pdpDisconnected,
            notLoggingToFlashdrive;

    // intake alerts
    public static final Alert rollerConfigFail, rollerDisconnected, rollerOverheating, rollerFaults, rollerWarnings;

    // indexer alerts
    public static final Alert spindexerDisconnected,
            spindexerOverheating,
            spindexerFaults,
            spindexerWarnings,
            spindexerConfigFail;
    public static final Alert exchangeConfigFail,
            exchangeDisconnected,
            exchangeOverheating,
            exchangeFaults,
            exchangeWarnings;

    // climber alerts
    public static final Alert climberDisconnected, climberOverheating, climberFaults, climberConfigFail;

    // shooter alerts
    public static final Alert shooterDisconnected,
            shooterOverheating,
            shooterFaults,
            shooterWarnings,
            shooterConfigFail;

    // turret alerts
    public static final Alert turretDisconnected, turretOverheating, turretFaults, turretWarnings, turretConfigFail;

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

        pdpDisconnected = new Alert("Missing connection to PDP", AlertType.kError);
        pdpDisconnected.set(false);

        notLoggingToFlashdrive = new Alert(
                "Logger is not logging to a flash drive. Please confirm that the flash drive is securely plugged in and was plugged in before the robot was turned on",
                AlertType.kError);
        notLoggingToFlashdrive.set(false);

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

        spindexerDisconnected = new Alert(
                String.format("Missing connection to spindexer motor (CAN %d)", IndexerConstants.Spindexer.motorID),
                AlertType.kError);
        spindexerDisconnected.set(false);

        spindexerOverheating = new Alert(
                String.format("Spindexer motor (CAN %d) is overheating", IndexerConstants.Spindexer.motorID),
                AlertType.kWarning);
        spindexerOverheating.set(false);

        spindexerFaults = new Alert(
                String.format(
                        "Potentially critical faults are active on spindexer motor (CAN %d)",
                        IndexerConstants.Spindexer.motorID),
                AlertType.kWarning);
        spindexerFaults.set(false);

        spindexerWarnings = new Alert(
                String.format(
                        "Potentially critical warnings are active on spindexer motor (CAN %d)",
                        IndexerConstants.Spindexer.motorID),
                AlertType.kWarning);
        spindexerWarnings.set(false);

        spindexerConfigFail = new Alert(
                String.format("Failed to update spindexer motor (CAN %d) config", IndexerConstants.Spindexer.motorID),
                AlertType.kError);
        spindexerConfigFail.set(false);

        exchangeConfigFail = new Alert(
                String.format("Failed to update exchange motor (CAN %d) config", IndexerConstants.Exchange.motorID),
                AlertType.kError);
        exchangeConfigFail.set(false);

        exchangeDisconnected = new Alert(
                String.format("Missing connection to exchange motor (CAN %d)", IndexerConstants.Exchange.motorID),
                AlertType.kError);
        exchangeDisconnected.set(false);

        exchangeOverheating = new Alert(
                String.format("Exchange motor (CAN %d) is overheating", IndexerConstants.Exchange.motorID),
                AlertType.kWarning);
        exchangeOverheating.set(false);

        exchangeFaults = new Alert(
                String.format(
                        "Potentially critical faults are active on exchange motor (CAN %d)",
                        IndexerConstants.Exchange.motorID),
                AlertType.kWarning);
        exchangeFaults.set(false);

        exchangeWarnings = new Alert(
                String.format(
                        "Potentially critical warnings are active on exchange motor (CAN %d)",
                        IndexerConstants.Exchange.motorID),
                AlertType.kWarning);
        exchangeWarnings.set(false);

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

        shooterDisconnected = new Alert(
                String.format("Missing connection to shooter motor (CAN %d)", ShooterConstants.ShootConfig.motorID),
                AlertType.kError);
        shooterDisconnected.set(false);

        shooterOverheating = new Alert(
                String.format("Shooter motor (CAN %d) is overheating", ShooterConstants.ShootConfig.motorID),
                AlertType.kWarning);
        shooterOverheating.set(false);

        shooterFaults = new Alert(
                String.format(
                        "Potentially critical faults are active on shooter motor (CAN %d)",
                        ShooterConstants.ShootConfig.motorID),
                AlertType.kWarning);
        shooterFaults.set(false);

        shooterWarnings = new Alert(
                String.format(
                        "Potentially critical warnings are active on shooter motor (CAN %d)",
                        ShooterConstants.ShootConfig.motorID),
                AlertType.kWarning);
        shooterWarnings.set(false);

        shooterConfigFail = new Alert(
                String.format("Failed to update shooter motor (CAN %d) config", ShooterConstants.ShootConfig.motorID),
                AlertType.kError);
        shooterConfigFail.set(false);

        turretDisconnected = new Alert(
                String.format("Missing connection to turret motor (CAN %d)", TurretConstants.motorID),
                AlertType.kError);
        turretDisconnected.set(false);

        turretOverheating = new Alert(
                String.format("Turret motor (CAN %d) is overheating", TurretConstants.motorID), AlertType.kWarning);
        turretOverheating.set(false);

        turretFaults = new Alert(
                String.format(
                        "Potentially critical faults are active on turret motor (CAN %d)", TurretConstants.motorID),
                AlertType.kWarning);
        turretFaults.set(false);

        turretWarnings = new Alert(
                String.format(
                        "Potentially critical warnings are active on turret motor (CAN %d)", TurretConstants.motorID),
                AlertType.kWarning);
        turretWarnings.set(false);

        turretConfigFail = new Alert(
                String.format("Failed to update turret motor (CAN %d) config", TurretConstants.motorID),
                AlertType.kError);
        turretConfigFail.set(false);
    }

    private Alerts() {}
}
