package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.utils.Logger;
import frc.robot.utils.TalonFXUtils;
import java.util.Optional;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX climber;

    public ClimbSubsystem() {
        climber = new TalonFX(ClimberConstants.motorID);
        configureTalonFXMotor();
    }

    @Override
    public void periodic() {
        Logger.logTalonFX("Climber", "motor", climber, Optional.empty());
    }

    // example code to set the closed-loop (PID) target velocity to a given value
    // this function is public so that it can be called from commands using this subsystem
    public void setClimbMotorPosition(double Position) {
        PositionVoltage request = new PositionVoltage(Position / 360);
        climber.setControl(request);
    }

    // function to immediately stop all physical movement in the subsystem
    // this is used mostly for safety reasons, such as to stop all movement on the robot when it is disabled
    // this would usually be called in Robot.disabledInit
    public void stop() {
        climber.stopMotor();
    }

    public double getPosition() {
        return climber.getPosition().getValue().abs(Units.Degrees);
    }

    // this is a function for use only within the subsystem itself, so it is marked private
    // this is a function to configure/setup a motor when the robot turns on
    private void configureTalonFXMotor() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        TalonFXUtils.configureBasicSettings(
                motorConfig,
                ClimberConstants.maxCurrent,
                ClimberConstants.neutralMode,
                ClimberConstants.inverted,
                ClimberConstants.maxDutyCycle,
                ClimberConstants.maxVoltage);
        TalonFXUtils.configureClosedLoopSettings(
                motorConfig,
                ClimberConstants.kP,
                ClimberConstants.kI,
                ClimberConstants.kD,
                ClimberConstants.kG,
                ClimberConstants.kGType);

        climber.getConfigurator().apply(motorConfig);
    }
}
