package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotContainer;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.generated.TunerConstants;

public class Alerts {

    // general alerts
    public static final Alert driver1Missing,
            driver2Missing,
            buttonBoxConnected,
            fmsConnected,
            lowBattery,
            criticalBattery,
            pdpDisconnected,
            notLoggingToFlashdrive;

    // intake alerts
    public static final Alert rollerConfigFail, rollerDisconnected, rollerOverheating, rollerFaults;

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

    // swerve alerts
    public static final Alert flDriveDisconnected, flDriveOverheating;
    public static final Alert flAzimuthDisconnected, flAzimuthOverheating;
    public static final Alert frDriveDisconnected, frDriveOverheating;
    public static final Alert frAzimuthDisconnected, frAzimuthOverheating;
    public static final Alert blDriveDisconnected, blDriveOverheating;
    public static final Alert blAzimuthDisconnected, blAzimuthOverheating;
    public static final Alert brDriveDisconnected, brDriveOverheating;
    public static final Alert brAzimuthDisconnected, brAzimuthOverheating;

    static {
        driver1Missing = new Alert("Driver 1 controller is not plugged in to port 0", AlertType.kWarning);
        driver2Missing = new Alert("Driver 2 controller is not plugged in to port 1", AlertType.kWarning);
        buttonBoxConnected = new Alert("Manual control button box is connected", AlertType.kInfo);

        fmsConnected = new Alert("FMS connected", AlertType.kInfo);

        lowBattery = new Alert("Low battery", AlertType.kWarning);
        criticalBattery = new Alert("Very low battery", AlertType.kError);

        notLoggingToFlashdrive = new Alert(
                "Logger is not logging to a flash drive. Please confirm that the flash drive is securely plugged in and was plugged in before the robot was turned on",
                AlertType.kError);

        rollerConfigFail = makeConfigFailAlert("roller motor", IntakeConstants.Roller.motorID);
        rollerDisconnected = makeDisconnectAlert("roller motor", IntakeConstants.Roller.motorID);
        rollerOverheating = makeOverheatingAlert("roller motor", IntakeConstants.Roller.motorID);
        rollerFaults = makeCriticalFaultsAlert("roller motor", IntakeConstants.Roller.motorID);

        spindexerDisconnected = makeDisconnectAlert("spindexer motor", IndexerConstants.Spindexer.motorID);
        spindexerOverheating = makeOverheatingAlert("spindexer motor", IndexerConstants.Spindexer.motorID);
        spindexerFaults = makeCriticalFaultsAlert("spindexer motor", IndexerConstants.Spindexer.motorID);
        spindexerWarnings = makeCriticalWarningsAlert("spindexer motor", IndexerConstants.Spindexer.motorID);
        spindexerConfigFail = makeConfigFailAlert("spindexer motor", IndexerConstants.Spindexer.motorID);

        exchangeConfigFail = makeConfigFailAlert("exchange motor", IndexerConstants.Exchange.motorID);
        exchangeDisconnected = makeDisconnectAlert("exchange motor", IndexerConstants.Exchange.motorID);
        exchangeOverheating = makeOverheatingAlert("exchange motor", IndexerConstants.Exchange.motorID);
        exchangeFaults = makeCriticalFaultsAlert("exchange motor", IndexerConstants.Exchange.motorID);
        exchangeWarnings = makeCriticalWarningsAlert("exchange motor", IndexerConstants.Exchange.motorID);

        climberDisconnected = makeDisconnectAlert("climber motor", ClimberConstants.motorID);
        climberOverheating = makeOverheatingAlert("climber motor", ClimberConstants.motorID);
        climberFaults = makeCriticalFaultsAlert("climber motor", ClimberConstants.motorID);
        climberConfigFail = makeConfigFailAlert("climber motor", ClimberConstants.motorID);

        shooterDisconnected = makeDisconnectAlert("shooter motor", ShooterConstants.motorID);
        shooterOverheating = makeOverheatingAlert("shooter motor", ShooterConstants.motorID);
        shooterFaults = makeCriticalFaultsAlert("shooter motor", ShooterConstants.motorID);
        shooterWarnings = makeCriticalWarningsAlert("shooter motor", ShooterConstants.motorID);
        shooterConfigFail = makeConfigFailAlert("shooter motor", ShooterConstants.motorID);

        turretDisconnected = makeDisconnectAlert("turret motor", TurretConstants.motorID);
        turretOverheating = makeOverheatingAlert("turret motor", TurretConstants.motorID);
        turretFaults = makeCriticalFaultsAlert("turret motor", TurretConstants.motorID);
        turretWarnings = makeCriticalWarningsAlert("turret motor", TurretConstants.motorID);
        turretConfigFail = makeConfigFailAlert("turret motor", TurretConstants.motorID);

        flDriveDisconnected = makeDisconnectAlert("flDrive motor", TunerConstants.FrontLeft.DriveMotorId);
        flDriveOverheating = makeOverheatingAlert("flDrive motor", TunerConstants.FrontLeft.DriveMotorId);
        flAzimuthDisconnected = makeDisconnectAlert("flAzimuth motor", TunerConstants.FrontLeft.SteerMotorId);
        flAzimuthOverheating = makeOverheatingAlert("flAzimuth motor", TunerConstants.FrontLeft.SteerMotorId);
        frDriveDisconnected = makeDisconnectAlert("frDrive motor", TunerConstants.FrontRight.DriveMotorId);
        frDriveOverheating = makeOverheatingAlert("frDrive motor", TunerConstants.FrontRight.DriveMotorId);
        frAzimuthDisconnected = makeDisconnectAlert("frAzimuth motor", TunerConstants.FrontRight.SteerMotorId);
        frAzimuthOverheating = makeOverheatingAlert("frAzimuth motor", TunerConstants.FrontRight.SteerMotorId);
        blDriveDisconnected = makeDisconnectAlert("blDrive motor", TunerConstants.BackLeft.DriveMotorId);
        blDriveOverheating = makeOverheatingAlert("blDrive motor", TunerConstants.BackLeft.DriveMotorId);
        blAzimuthDisconnected = makeDisconnectAlert("blAzimuth motor", TunerConstants.BackLeft.SteerMotorId);
        blAzimuthOverheating = makeOverheatingAlert("blAzimuth motor", TunerConstants.BackLeft.SteerMotorId);
        brDriveDisconnected = makeDisconnectAlert("brDrive motor", TunerConstants.BackRight.DriveMotorId);
        brDriveOverheating = makeOverheatingAlert("brDrive motor", TunerConstants.BackRight.DriveMotorId);
        brAzimuthDisconnected = makeDisconnectAlert("brAzimuth motor", TunerConstants.BackRight.SteerMotorId);
        brAzimuthOverheating = makeOverheatingAlert("brAzimuth motor", TunerConstants.BackRight.SteerMotorId);

        // ALWAYS RUN LAST
        pdpDisconnected =
                makeDisconnectAlert("PDP", RobotContainer.getInstance().pdp.getModule() + 1);
    }

    private Alerts() {}

    private static Alert makeConfigFailAlert(String deviceName, int can) {
        return new Alert(String.format("Failed to update %s (CAN %d) config", deviceName, can), AlertType.kError);
    }

    private static Alert makeDisconnectAlert(String deviceName, int can) {
        return new Alert(String.format("Missing connection to %s (CAN %d)", deviceName, can), AlertType.kError);
    }

    private static Alert makeOverheatingAlert(String deviceName, int can) {
        return new Alert(String.format("%s (CAN %d) is overheating", deviceName, can), AlertType.kWarning);
    }

    private static Alert makeCriticalFaultsAlert(String deviceName, int can) {
        return new Alert(
                String.format("Potentially critical faults are active on %s (CAN %d)", deviceName, can),
                AlertType.kWarning);
    }

    private static Alert makeCriticalWarningsAlert(String deviceName, int can) {
        return new Alert(
                String.format("Potentially critical warnings are active on %s (CAN %d)", deviceName, can),
                AlertType.kWarning);
    }
}
