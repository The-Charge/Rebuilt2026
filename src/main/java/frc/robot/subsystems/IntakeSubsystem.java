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

    enum State {
        DEPLOYED,
        RETRACTED,
        UNKNOWN,
    }

    public State state; // TODO: use this???

    public IntakeSubsystem() {
        pivotMotor = new TalonFX(IntakeConstants.PivotMotorId);
        rollerMotor = new SparkMax(IntakeConstants.RollerMotorId, MotorType.kBrushless);
        
        state = State.RETRACTED;
    }

    public boolean isDeployed() { return state == State.DEPLOYED; }

    public boolean isRetracted() { return state == State.RETRACTED; }

    public void deploy() {
        if (isDeployed()) return;
        
        PositionVoltage request = new PositionVoltage(IntakeConstants.DeployedPosition);
        pivotMotor.setControl(request);
    }

    public void retract() {
        if (isRetracted()) return;

        PositionVoltage request = new PositionVoltage(IntakeConstants.RetractedPosition);
        pivotMotor.setControl(request);
    }

    public void startIntake() {
        rollerMotor
            .getClosedLoopController() // this might cuase problems??????
            .setSetpoint(IntakeConstants.RollerVelocity, ControlType.kVelocity);
    }

    public void stopIntake() {
        rollerMotor.set(0);
    }

    @Override
    public void periodic() {
        
    }
}
