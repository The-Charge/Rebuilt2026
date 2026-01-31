package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.Roller;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax rollerMotor;

    private ServoHub deployerServoHub;
    private ServoChannel deployerServoChannel;

    private boolean deployed;

    public IntakeSubsystem() {
        rollerMotor = new SparkMax(IntakeConstants.Roller.motorID, MotorType.kBrushless);

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
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        SparkUtils.configureBasicSettings(
                motorConfig,
                Roller.currentLimit,
                Roller.idleMode,
                Roller.inverted,
                Roller.maxDutyCycle,
                Roller.nominalVoltage);

        if (rollerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
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
        Logger.logSparkMotor(IntakeConstants.name, "Roller", rollerMotor);
        Logger.logBool(IntakeConstants.name, "isDeployed", deployed);

        Alerts.rollerDisconnected.set(!SparkUtils.isConnected(rollerMotor));
        Alerts.rollerOverheating.set(rollerMotor.getMotorTemperature() >= 80);
        Alerts.rollerFaults.set(SparkUtils.hasCriticalFaults(rollerMotor.getFaults()));
        Alerts.rollerWarnings.set(SparkUtils.hasCriticalWarnings(rollerMotor.getWarnings()));
    }
}
