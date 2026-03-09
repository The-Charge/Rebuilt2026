package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.Roller;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.TalonFXUtils;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor;
    private final Servo leftDeployer, rightDeployer;

    private boolean deployed;

    public IntakeSubsystem() {
        rollerMotor = new TalonFX(IntakeConstants.Roller.motorID);
        leftDeployer = new Servo(IntakeConstants.LeftDeployer.port);
        rightDeployer = new Servo(IntakeConstants.RightDeployer.port);

        deployed = false;

        configureRollerMotor();
    }

    public void deploy() {
        leftDeployer.set(IntakeConstants.LeftDeployer.deployedPosition);
        rightDeployer.set(IntakeConstants.RightDeployer.deployedPosition);
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
            Logger.reportError("Failed to configure roller motor");
            Alerts.rollerConfigFail.set(true);
        }
    }

    public boolean isDeployed() {
        return deployed;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(IntakeConstants.name, this);

        Logger.logTalonFX(IntakeConstants.name, "rollerMotor", rollerMotor);

        Logger.logServo(IntakeConstants.name, "leftDeployer", leftDeployer);
        Logger.logServo(IntakeConstants.name, "rightDeployer", rightDeployer);

        Logger.logBool(IntakeConstants.name, "isDeployed", deployed);
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean rollerConnected = rollerMotor.isConnected();

        CANMonitor.logCANDeviceStatus("rollerMotor", IntakeConstants.Roller.motorID, rollerConnected);
        Alerts.rollerDisconnected.set(!rollerConnected);
        Alerts.rollerOverheating.set(rollerMotor.getDeviceTemp().getValue().in(Celsius) >= Roller.overheatingTemp);
        Alerts.rollerFaults.set(TalonFXUtils.getAllActiveFaults(rollerMotor).hasCriticalFaults());
    }
}
