package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.Roller;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.ServoUtils;
import frc.robot.utils.TalonFXUtils;
import java.util.Optional;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX rollerMotor;

    private final ServoHub deployerServoHub;
    private final ServoChannel deployerServoChannel;

    private boolean deployed;

    public IntakeSubsystem() {
        rollerMotor = new TalonFX(IntakeConstants.Roller.motorID);

        deployed = false;

        deployerServoHub = new ServoHub(IntakeConstants.Servo.servoID);
        deployerServoChannel = deployerServoHub.getServoChannel(IntakeConstants.Servo.channelId);

        configureRollerMotor();
    }

    public void deploy() {
        deployerServoChannel.setPowered(true);
        deployerServoChannel.setEnabled(true);

        deployerServoChannel.setPulseWidth(IntakeConstants.Servo.deployedPulseWidth);
        deployed = true;
    }

    public void startRoller(double speed) {
        rollerMotor.set(speed);
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
                Roller.voltageLimit);

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
        Logger.logTalonFX(IntakeConstants.name, "Roller", rollerMotor);
        Logger.logBool(IntakeConstants.name, "isDeployed", deployed);
        Logger.logServoHub(IntakeConstants.name, "ServoHub", deployerServoHub, Optional.empty());

        Alerts.rollerDisconnected.set(!rollerMotor.isConnected());
        Alerts.rollerOverheating.set(rollerMotor.getDeviceTemp().getValue().abs(Celsius) >= Roller.overheatingTemp);
        Alerts.rollerFaults.set(TalonFXUtils.getAllActiveFaults(rollerMotor).hasCriticalFaults());
        Alerts.rollerStickyFaults.set(
                TalonFXUtils.getAllStickyFaults(rollerMotor).hasCriticalFaults());

        Alerts.servoDisconnected.set(!ServoUtils.isConnected(deployerServoHub));
        Alerts.servoFaults.set(ServoUtils.hasCriticalFaults(deployerServoHub.getFaults()));
        Alerts.servoWarnings.set(ServoUtils.hasCriticalWarnings(deployerServoHub.getWarnings()));
    }
}
