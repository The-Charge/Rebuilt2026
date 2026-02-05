package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants.Roller;
import frc.robot.constants.IntakeConstants.Servo;

public class Alerts {

    // general alerts
    public static final Alert driver1Missing, driver2Missing, fmsConnected, lowBattery, criticalBattery;

    // intake alerts
    public static final Alert rollerConfigFail, rollerDisconnected, rollerOverheating, rollerFaults, rollerWarnings;
    public static final Alert servoDisconnected, servoFaults, servoWarnings;

    // indexer alerts
    public static final Alert spindexerDisconnected,
            spindexerOverheating,
            spindexerFaults,
            spindexerWarnings,
            spindexerConfigFail;
    public static final Alert gateConfigFail, gateDisconnected, gateOverheating, gateFaults, gateWarnings;

    // climber alerts
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

        servoDisconnected =
                new Alert(String.format("Missing connection to servo hub (CAN %d)", Servo.servoID), AlertType.kError);
        servoDisconnected.set(false);

        servoFaults = new Alert(
                String.format("Potentially critical faults are active on servo hub (CAN %d)", Servo.servoID),
                AlertType.kWarning);
        servoFaults.set(false);

        servoWarnings = new Alert(
                String.format("Potentially critical warnings are active on servo hub (CAN %d)", Servo.servoID),
                AlertType.kWarning);
        servoWarnings.set(false);

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

        gateConfigFail = new Alert(
                String.format("Failed to update gate motor (CAN %d) config", IndexerConstants.Gate.motorID),
                AlertType.kError);
        gateConfigFail.set(false);

        gateDisconnected = new Alert(
                String.format("Missing connection to gate motor (CAN %d)", IndexerConstants.Gate.motorID),
                AlertType.kError);
        gateDisconnected.set(false);

        gateOverheating = new Alert(
                String.format("Gate motor (CAN %d) is overheating", IndexerConstants.Gate.motorID), AlertType.kWarning);
        gateOverheating.set(false);

        gateFaults = new Alert(
                String.format(
                        "Potentially critical faults are active on gate motor (CAN %d)", IndexerConstants.Gate.motorID),
                AlertType.kWarning);
        gateFaults.set(false);

        gateWarnings = new Alert(
                String.format(
                        "Potentially critical warnings are active on gate motor (CAN %d)",
                        IndexerConstants.Gate.motorID),
                AlertType.kWarning);
        gateWarnings.set(false);

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
