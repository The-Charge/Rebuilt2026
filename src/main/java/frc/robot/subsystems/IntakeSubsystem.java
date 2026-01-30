package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    public SparkMax rollerMotor;

    public ServoHub deployerServoHub;
    public ServoChannel deployerServoChannel;

    public boolean deployed;

    public IntakeSubsystem() {
        rollerMotor = new SparkMax(IntakeConstants.RollerMotorId, MotorType.kBrushless);

        deployed = false;

        deployerServoHub = new ServoHub(IntakeConstants.deployerServoID);
        deployerServoChannel = deployerServoHub.getServoChannel(ChannelId.kChannelId0);

        configureRollerMotor();
    }

    public void deploy() {
        deployerServoChannel.setPowered(true);
        deployerServoChannel.setEnabled(true);

        deployerServoChannel.setPulseWidth(IntakeConstants.deployedPulseWidth);
        deployed = true;
    }

    public void startRoller() {
        rollerMotor.set(1.00);
    }

    public void stopRoller() {
        rollerMotor.set(0);
    }

    public void configureRollerMotor() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(IntakeConstants.currentLimit);
        rollerMotor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters); // TODO: I don't know if these are good params
    }

    @Override
    public void periodic() {}
}
