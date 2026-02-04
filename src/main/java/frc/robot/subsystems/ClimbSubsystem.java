package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.units.ClimberPosition;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.TalonFXUtils;
import java.util.Optional;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX motor;

    private Optional<ClimberPosition> motorTarget;

    public ClimbSubsystem() {
        motor = new TalonFX(ClimberConstants.motorID);
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
        if (motor.getConfigurator().apply(motorConfig) != StatusCode.OK) {
            Logger.reportError("Failed to configure climber motor");
            Alerts.climberConfigFail.set(true);
        }

        motorTarget = Optional.empty();
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(ClimberConstants.subsystemName, this);

        Logger.logTalonFX(ClimberConstants.subsystemName, "motor", motor);
        Alerts.climberDisconnected.set(!motor.isConnected());
        Alerts.climberOverheating.set(motor.getDeviceTemp().getValue().abs(Units.Celsius) >= 80);
        Alerts.climberFaults.set(TalonFXUtils.getAllActiveFaults(motor).hasCriticalFaults());

        Logger.logDouble(
                ClimberConstants.subsystemName,
                "motor/targetMotorRots",
                motorTarget.map((val) -> val.asMotorRotations()).orElse(Double.NaN));
        Logger.logBool(
                ClimberConstants.subsystemName,
                "motor/isAtTarget",
                isMotorAtTarget().orElse(true));
    }

    public void setPosition(ClimberPosition position) {
        if (position == null) {
            Logger.reportWarning("Cannot move climber to a null ClimberPosition", true);
            return;
        }

        motorTarget = Optional.of(position);

        PositionVoltage request = new PositionVoltage(position.asMotorRotations());
        motor.setControl(request);
    }

    public void stopAll() {
        motorTarget = Optional.empty();

        motor.stopMotor();
    }

    public ClimberPosition getPosition() {
        return ClimberPosition.fromMotorRotations(motor.getPosition().getValue().abs(Units.Rotations));
    }

    public Optional<Boolean> isMotorAtTarget() {
        if (motorTarget.isEmpty()) return Optional.empty();

        double motorRots = getPosition().asMotorRotations();
        double targetRots = motorTarget.get().asMotorRotations();
        double toleranceRots = ClimberConstants.targetTolerance.asMotorRotations();

        return Optional.of(Math.abs(targetRots - motorRots) <= toleranceRots);
    }
}
