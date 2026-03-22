package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.LeftDeployer;
import frc.robot.constants.IntakeConstants.RightDeployer;
import frc.robot.constants.IntakeConstants.Roller;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.TalonFXUtils;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor;
    private final Servo leftDeployer, rightDeployer;
    private final Alert rollerConfigFail, rollerDisconnected, rollerOverheating, rollerFaults;

    private boolean deployed;

    public IntakeSubsystem() {
        rollerConfigFail = Alerts.makeConfigFailAlert(Roller.motorName, Roller.motorID);
        rollerDisconnected = Alerts.makeDisconnectAlert(Roller.motorName, Roller.motorID);
        rollerOverheating = Alerts.makeOverheatingAlert(Roller.motorName, Roller.motorID);
        rollerFaults = Alerts.makeCriticalFaultsAlert(Roller.motorName, Roller.motorID);

        rollerMotor = new TalonFX(Roller.motorID);
        leftDeployer = new Servo(LeftDeployer.port);
        rightDeployer = new Servo(RightDeployer.port);

        deployed = false;

        configureRollerMotor();
    }

    @Override
    public String getName() {
        return IntakeConstants.subsystemName;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(getName(), this);

        Logger.logTalonFX(getName(), Roller.motorName, rollerMotor);

        Logger.logServo(getName(), LeftDeployer.servoName, leftDeployer);
        Logger.logServo(getName(), RightDeployer.servoName, rightDeployer);

        Logger.logBool(getName(), "isDeployed", deployed);
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean rollerConnected = rollerMotor.isConnected();

        CANMonitor.logCANDeviceStatus(Roller.motorName, Roller.motorID, rollerConnected);
        rollerDisconnected.set(!rollerConnected);
        rollerOverheating.set(rollerMotor.getDeviceTemp().getValue().in(Celsius) >= 80);
        rollerFaults.set(TalonFXUtils.getAllActiveFaults(rollerMotor).hasCriticalFaults());
    }

    public void deploy() {
        leftDeployer.set(LeftDeployer.deployedPosition);
        rightDeployer.set(RightDeployer.deployedPosition);
        deployed = true;
    }

    public void setRollerVoltage(double volts) {
        rollerMotor.setVoltage(volts);
    }

    public void stopRoller() {
        rollerMotor.set(0);
    }

    private void configureRollerMotor() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        TalonFXUtils.configureBasicSettings(
                motorConfig,
                Roller.currentLimit,
                Roller.neutralMode,
                Roller.inverted,
                Roller.maxDutyCycle,
                Roller.maxVoltage);

        if (rollerMotor.getConfigurator().apply(motorConfig) != StatusCode.OK) {
            Logger.reportError(String.format("Failed to configure %s", Roller.motorName));
            rollerConfigFail.set(true);
        }
    }

    public boolean isDeployed() {
        return deployed;
    }
}
