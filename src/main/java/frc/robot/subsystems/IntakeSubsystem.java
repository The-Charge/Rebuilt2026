package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

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

    public boolean isDeployed() { return currentState() == Target.DEPLOYED; }
    public boolean isRetracted() { return currentState() == Target.RETRACTED; }

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

    @Override
    public void periodic() {
        
    }
}
