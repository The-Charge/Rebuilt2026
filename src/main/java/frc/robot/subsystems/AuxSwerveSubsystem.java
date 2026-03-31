package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;

public class AuxSwerveSubsystem extends SubsystemBase {

    private final Alert flDriveDisconnected, flDriveOverheating;
    private final Alert flAzimuthDisconnected, flAzimuthOverheating;
    private final Alert frDriveDisconnected, frDriveOverheating;
    private final Alert frAzimuthDisconnected, frAzimuthOverheating;
    private final Alert blDriveDisconnected, blDriveOverheating;
    private final Alert blAzimuthDisconnected, blAzimuthOverheating;
    private final Alert brDriveDisconnected, brDriveOverheating;
    private final Alert brAzimuthDisconnected, brAzimuthOverheating;
    private final Alert pigeonDisconnected;

    public AuxSwerveSubsystem() {
        flDriveDisconnected =
                Alerts.makeDisconnectAlert(SwerveConstants.flDriveName, TunerConstants.FrontLeft.DriveMotorId);
        flDriveOverheating =
                Alerts.makeOverheatingAlert(SwerveConstants.flDriveName, TunerConstants.FrontLeft.DriveMotorId);
        flAzimuthDisconnected =
                Alerts.makeDisconnectAlert(SwerveConstants.flAzimuthName, TunerConstants.FrontLeft.SteerMotorId);
        flAzimuthOverheating =
                Alerts.makeOverheatingAlert(SwerveConstants.flAzimuthName, TunerConstants.FrontLeft.SteerMotorId);
        frDriveDisconnected =
                Alerts.makeDisconnectAlert(SwerveConstants.frDriveName, TunerConstants.FrontRight.DriveMotorId);
        frDriveOverheating =
                Alerts.makeOverheatingAlert(SwerveConstants.frDriveName, TunerConstants.FrontRight.DriveMotorId);
        frAzimuthDisconnected =
                Alerts.makeDisconnectAlert(SwerveConstants.frAzimuthName, TunerConstants.FrontRight.SteerMotorId);
        frAzimuthOverheating =
                Alerts.makeOverheatingAlert(SwerveConstants.frAzimuthName, TunerConstants.FrontRight.SteerMotorId);
        blDriveDisconnected =
                Alerts.makeDisconnectAlert(SwerveConstants.blDriveName, TunerConstants.BackLeft.DriveMotorId);
        blDriveOverheating =
                Alerts.makeOverheatingAlert(SwerveConstants.blDriveName, TunerConstants.BackLeft.DriveMotorId);
        blAzimuthDisconnected =
                Alerts.makeDisconnectAlert(SwerveConstants.blAzimuthName, TunerConstants.BackLeft.SteerMotorId);
        blAzimuthOverheating =
                Alerts.makeOverheatingAlert(SwerveConstants.blAzimuthName, TunerConstants.BackLeft.SteerMotorId);
        brDriveDisconnected =
                Alerts.makeDisconnectAlert(SwerveConstants.brDriveName, TunerConstants.BackRight.DriveMotorId);
        brDriveOverheating =
                Alerts.makeOverheatingAlert(SwerveConstants.brDriveName, TunerConstants.BackRight.DriveMotorId);
        brAzimuthDisconnected =
                Alerts.makeDisconnectAlert(SwerveConstants.brAzimuthName, TunerConstants.BackRight.SteerMotorId);
        brAzimuthOverheating =
                Alerts.makeOverheatingAlert(SwerveConstants.brAzimuthName, TunerConstants.BackRight.SteerMotorId);
        pigeonDisconnected =
                Alerts.makeDisconnectAlert(SwerveConstants.pigeonName, TunerConstants.DrivetrainConstants.Pigeon2Id);
    }

    @Override
    public void periodic() {
        CommandSwerveDrivetrain swerve = RobotContainer.getInstance().swerve;
        Logger.logSubsystem(getName(), swerve);

        Logger.logTalonFXReduced(getName(), SwerveConstants.flDriveName, flDrive());
        Logger.logTalonFXReduced(getName(), SwerveConstants.flAzimuthName, flAzimuth());
        Logger.logTalonFXReduced(getName(), SwerveConstants.frDriveName, frDrive());
        Logger.logTalonFXReduced(getName(), SwerveConstants.frAzimuthName, frAzimuth());
        Logger.logTalonFXReduced(getName(), SwerveConstants.blDriveName, blDrive());
        Logger.logTalonFXReduced(getName(), SwerveConstants.blAzimuthName, blAzimuth());
        Logger.logTalonFXReduced(getName(), SwerveConstants.brDriveName, brDrive());
        Logger.logTalonFXReduced(getName(), SwerveConstants.brAzimuthName, brAzimuth());
        Logger.logPigeon2(getName(), SwerveConstants.pigeonName, pigeon());

        ChassisSpeeds chassisSpeeds = swerve.getState().Speeds;
        Logger.logDouble(
                getName(), "chassisVel", Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
    }

    @Override
    public String getName() {
        return SwerveConstants.subsystemName;
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
        boolean pigeonConnected = pigeon().isConnected();

        CANMonitor.logCANDeviceStatus(
                SwerveConstants.flDriveName, TunerConstants.FrontLeft.DriveMotorId, flDriveConnected);
        flDriveDisconnected.set(!flDriveConnected);
        flDriveOverheating.set(flDrive().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus(
                SwerveConstants.flAzimuthName, TunerConstants.FrontLeft.SteerMotorId, flAzimuthConnected);
        flAzimuthDisconnected.set(!flAzimuthConnected);
        flAzimuthOverheating.set(flAzimuth().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus(
                SwerveConstants.frDriveName, TunerConstants.FrontRight.DriveMotorId, frDriveConnected);
        frDriveDisconnected.set(!frDriveConnected);
        frDriveOverheating.set(frDrive().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus(
                SwerveConstants.frAzimuthName, TunerConstants.FrontRight.SteerMotorId, frAzimuthConnected);
        frAzimuthDisconnected.set(!frAzimuthConnected);
        frAzimuthOverheating.set(frAzimuth().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus(
                SwerveConstants.blDriveName, TunerConstants.BackLeft.DriveMotorId, blDriveConnected);
        blDriveDisconnected.set(!blDriveConnected);
        blDriveOverheating.set(blDrive().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus(
                SwerveConstants.blAzimuthName, TunerConstants.BackLeft.SteerMotorId, blAzimuthConnected);
        blAzimuthDisconnected.set(!blAzimuthConnected);
        blAzimuthOverheating.set(blAzimuth().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus(
                SwerveConstants.brDriveName, TunerConstants.BackRight.DriveMotorId, brDriveConnected);
        brDriveDisconnected.set(!brDriveConnected);
        brDriveOverheating.set(brDrive().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus(
                SwerveConstants.brAzimuthName, TunerConstants.BackRight.SteerMotorId, brAzimuthConnected);
        brAzimuthDisconnected.set(!brAzimuthConnected);
        brAzimuthOverheating.set(brAzimuth().getDeviceTemp().getValue().in(Celsius) >= 80);

        CANMonitor.logCANDeviceStatus(
                SwerveConstants.pigeonName, TunerConstants.DrivetrainConstants.Pigeon2Id, pigeonConnected);
        pigeonDisconnected.set(!pigeonConnected);
    }

    private TalonFX flDrive() {
        return RobotContainer.getInstance()
                .swerve
                .getModule(SwerveConstants.flModuleIndex)
                .getDriveMotor();
    }

    private TalonFX flAzimuth() {
        return RobotContainer.getInstance()
                .swerve
                .getModule(SwerveConstants.flModuleIndex)
                .getSteerMotor();
    }

    private TalonFX frDrive() {
        return RobotContainer.getInstance()
                .swerve
                .getModule(SwerveConstants.frModuleIndex)
                .getDriveMotor();
    }

    private TalonFX frAzimuth() {
        return RobotContainer.getInstance()
                .swerve
                .getModule(SwerveConstants.frModuleIndex)
                .getSteerMotor();
    }

    private TalonFX blDrive() {
        return RobotContainer.getInstance()
                .swerve
                .getModule(SwerveConstants.blModuleIndex)
                .getDriveMotor();
    }

    private TalonFX blAzimuth() {
        return RobotContainer.getInstance()
                .swerve
                .getModule(SwerveConstants.blModuleIndex)
                .getSteerMotor();
    }

    private TalonFX brDrive() {
        return RobotContainer.getInstance()
                .swerve
                .getModule(SwerveConstants.brModuleIndex)
                .getDriveMotor();
    }

    private TalonFX brAzimuth() {
        return RobotContainer.getInstance()
                .swerve
                .getModule(SwerveConstants.brModuleIndex)
                .getSteerMotor();
    }

    private Pigeon2 pigeon() {
        return RobotContainer.getInstance().swerve.getPigeon2();
    }
}
