package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;

public class AuxSwerveSubsystem extends SubsystemBase {

    @Override
    public void periodic() {
        Logger.logSubsystem(SwerveConstants.subsystemName, RobotContainer.getInstance().swerve);

        Logger.logTalonFXReduced(SwerveConstants.subsystemName, "FLDrive", flDrive());
        Logger.logTalonFXReduced(SwerveConstants.subsystemName, "FLAzimuth", flAzimuth());
        Logger.logTalonFXReduced(SwerveConstants.subsystemName, "FRDrive", frDrive());
        Logger.logTalonFXReduced(SwerveConstants.subsystemName, "FRAzimuth", frAzimuth());
        Logger.logTalonFXReduced(SwerveConstants.subsystemName, "BLDrive", blDrive());
        Logger.logTalonFXReduced(SwerveConstants.subsystemName, "BLAzimuth", blAzimuth());
        Logger.logTalonFXReduced(SwerveConstants.subsystemName, "BRDrive", brDrive());
        Logger.logTalonFXReduced(SwerveConstants.subsystemName, "BRAzmith", brAzimuth());
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean flDriveConnected = flDrive().isConnected();
        boolean flAzimuthConnected = flAzimuth().isConnected();
        boolean frDriveConnected = frDrive().isConnected();
        boolean frAzimuthConnected = frAzimuth().isConnected();
        boolean blDriveConnected = blDrive().isConnected();
        boolean blAzimuthConnected = blAzimuth().isConnected();
        boolean brDriveConnected = brDrive().isConnected();
        boolean brAzimuthConnected = brAzimuth().isConnected();

        CANMonitor.logCANDeviceStatus("flDrive", TunerConstants.FrontLeft.DriveMotorId, flDriveConnected);
        Alerts.flDriveDisconnected.set(!flDriveConnected);
        Alerts.flDriveOverheating.set(flDrive().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus("flAzimuth", TunerConstants.FrontLeft.SteerMotorId, flAzimuthConnected);
        Alerts.flAzimuthDisconnected.set(!flAzimuthConnected);
        Alerts.flAzimuthOverheating.set(flAzimuth().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus("frDrive", TunerConstants.FrontRight.DriveMotorId, frDriveConnected);
        Alerts.frDriveDisconnected.set(!frDriveConnected);
        Alerts.frDriveOverheating.set(frDrive().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus("frAzimuth", TunerConstants.FrontRight.SteerMotorId, frAzimuthConnected);
        Alerts.frAzimuthDisconnected.set(!frAzimuthConnected);
        Alerts.frAzimuthOverheating.set(frAzimuth().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus("blDrive", TunerConstants.BackLeft.DriveMotorId, blDriveConnected);
        Alerts.blDriveDisconnected.set(!blDriveConnected);
        Alerts.blDriveOverheating.set(blDrive().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus("blAzimuth", TunerConstants.BackLeft.SteerMotorId, blAzimuthConnected);
        Alerts.blAzimuthDisconnected.set(!blAzimuthConnected);
        Alerts.blAzimuthOverheating.set(blAzimuth().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus("brDrive", TunerConstants.BackRight.DriveMotorId, brDriveConnected);
        Alerts.brDriveDisconnected.set(!brDriveConnected);
        Alerts.brDriveOverheating.set(brDrive().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus("brAzimuth", TunerConstants.BackRight.SteerMotorId, brAzimuthConnected);
        Alerts.brAzimuthDisconnected.set(!brAzimuthConnected);
        Alerts.brAzimuthOverheating.set(brAzimuth().getDeviceTemp().getValue().in(Celsius) >= 80);
    }

    private TalonFX flDrive() {
        return RobotContainer.getInstance().swerve.getModule(0).getDriveMotor();
    }

    private TalonFX flAzimuth() {
        return RobotContainer.getInstance().swerve.getModule(0).getSteerMotor();
    }

    private TalonFX frDrive() {
        return RobotContainer.getInstance().swerve.getModule(1).getDriveMotor();
    }

    private TalonFX frAzimuth() {
        return RobotContainer.getInstance().swerve.getModule(1).getSteerMotor();
    }

    private TalonFX blDrive() {
        return RobotContainer.getInstance().swerve.getModule(2).getDriveMotor();
    }

    private TalonFX blAzimuth() {
        return RobotContainer.getInstance().swerve.getModule(2).getSteerMotor();
    }

    private TalonFX brDrive() {
        return RobotContainer.getInstance().swerve.getModule(3).getDriveMotor();
    }

    private TalonFX brAzimuth() {
        return RobotContainer.getInstance().swerve.getModule(3).getSteerMotor();
    }
}
