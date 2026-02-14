package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.IndexerConstants;

public class Alerts {

    // general alerts
    public static final Alert driver1Missing, driver2Missing, fmsConnected, lowBattery, criticalBattery;

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
    }

    private Alerts() {}
}
