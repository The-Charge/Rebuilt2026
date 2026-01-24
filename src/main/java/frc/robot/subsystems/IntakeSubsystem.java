package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.IntakeConstants.PivotConfig;
import frc.robot.constants.IntakeConstants.RollerConfig;

public class IntakeSubsystem extends SubsystemBase {
    public TalonFX pivotMotor;
    public SparkMax rollerMotor;

    enum Target {
        DEPLOYED,
        RETRACTED,
        UNKNOWN,
    }

    public Target target; // TODO: use this???

    public IntakeSubsystem() {
        pivotMotor = new TalonFX(IntakeConstants.PivotMotorId);
        rollerMotor = new SparkMax(IntakeConstants.RollerMotorId, MotorType.kBrushless);

        // pivotMotor.getConfigurator().

        target = Target.UNKNOWN;
    }

    public Target currentState() {
        if (target == Target.DEPLOYED
                && pivotMotor.getPosition().isNear(IntakeConstants.DeployedPosition, IntakeConstants.StateTolerance)) {
            return Target.DEPLOYED;
        }
        if (target == Target.RETRACTED
                && pivotMotor.getPosition().isNear(IntakeConstants.RetractedPosition, IntakeConstants.StateTolerance)) {
            return Target.RETRACTED;
        }

        return Target.UNKNOWN;
    }

    public boolean isDeployed() {
        return currentState() == Target.DEPLOYED;
    }

    public boolean isRetracted() {
        return currentState() == Target.RETRACTED;
    }

    public void deploy() {
        if (isDeployed()) return;

        target = Target.DEPLOYED;

        PositionVoltage request = new PositionVoltage(IntakeConstants.DeployedPosition);
        pivotMotor.setControl(request);
    }

    public void retract() {
        if (isRetracted()) return;

        target = Target.RETRACTED;

        PositionVoltage request = new PositionVoltage(IntakeConstants.RetractedPosition);
        pivotMotor.setControl(request);
    }

    public void startRoller() {
        rollerMotor
                .getClosedLoopController() // this might cuase problems??????
                .setSetpoint(IntakeConstants.RollerVelocity, ControlType.kVelocity);
    }

    public void stopRoller() {
        rollerMotor.set(0);
    }

    public void configureRollerMotor() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop.p(RollerConfig.pid.p);
        motorConfig.closedLoop.i(RollerConfig.pid.i);
        motorConfig.closedLoop.d(RollerConfig.pid.d);
        rollerMotor.configure(
                motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters); // TODO: I don't know if these are good params
    }

    public void configurePivotMotor() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.PeakForwardDutyCycle = PivotConfig.maxVBus;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -PivotConfig.maxVBus;
        motorConfig.MotorOutput.withNeutralMode(PivotConfig.neutralMode);
        motorConfig.MotorOutput.Inverted = PivotConfig.inverted;

        motorConfig.CurrentLimits.StatorCurrentLimit = PivotConfig.maxCurrent;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        Slot0Configs slotConfigs = motorConfig.Slot0;
        slotConfigs.kS = slotConfigs.kV = 0;
        slotConfigs.kP = PivotConfig.pidf.p;
        slotConfigs.kI = PivotConfig.pidf.i;
        slotConfigs.kD = PivotConfig.pidf.d;
        slotConfigs.kG = PivotConfig.pidf.f;

        slotConfigs.GravityType = GravityTypeValue.Elevator_Static;

        // conservative motionmagic configs
        MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 80; // Target acceleration of 80 rps/s (1 second)
        motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 800 rps/s/s (0.2 seconds)

        // motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = ElevConstants.hardStopResetsEncoder;
        // motorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = ElevConstants.hardStopResetValue;

        pivotMotor.getConfigurator().apply(motorConfig);

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = softLimits.ReverseSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = PivotConfig.maxPosTicks;
        softLimits.ReverseSoftLimitThreshold = PivotConfig.minPosTicks;
        pivotMotor.getConfigurator().apply(softLimits);
    }

    @Override
    public void periodic() {}
}
