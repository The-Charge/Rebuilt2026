package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.units.ClimberPosition;
import frc.robot.utils.Alerts;
import frc.robot.utils.CANMonitor;
import frc.robot.utils.Logger;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.TalonFXUtils;
import java.util.Optional;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final TimeOfFlight towerSensor;

    private Optional<ClimberPosition> motorTarget;

    public ClimbSubsystem() {
        motor = new TalonFX(ClimberConstants.ClimbMotor.motorID);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        TalonFXUtils.configureBasicSettings(
                motorConfig,
                ClimberConstants.ClimbMotor.maxCurrent,
                ClimberConstants.ClimbMotor.neutralMode,
                ClimberConstants.ClimbMotor.inverted,
                ClimberConstants.ClimbMotor.maxDutyCycle,
                ClimberConstants.ClimbMotor.maxVoltage);
        TalonFXUtils.configureClosedLoopSettings(
                motorConfig,
                ClimberConstants.ClimbMotor.kP,
                ClimberConstants.ClimbMotor.kI,
                ClimberConstants.ClimbMotor.kD,
                ClimberConstants.ClimbMotor.kG,
                ClimberConstants.ClimbMotor.kGType);
        if (motor.getConfigurator().apply(motorConfig) != StatusCode.OK) {
            Logger.reportError("Failed to configure climber motor");
            Alerts.climberConfigFail.set(true);
        }

        towerSensor = new TimeOfFlight(ClimberConstants.TowerSensor.sensorID);
        towerSensor.setRangingMode(
                ClimberConstants.TowerSensor.rangingMode, ClimberConstants.TowerSensor.sampleTime.in(Milliseconds));

        motorTarget = Optional.empty();
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(ClimberConstants.subsystemName, this);

        Logger.logTalonFX(ClimberConstants.subsystemName, "motor", motor);
        Logger.logDouble(
                ClimberConstants.subsystemName,
                "targetMotorRots",
                motorTarget.map((val) -> val.asMotorRotations()).orElse(Double.NaN));
        Logger.logBool(
                ClimberConstants.subsystemName, "isAtTarget", isMotorAtTarget().orElse(true));

        Logger.logTOF(ClimberConstants.subsystemName, "towerSensor", towerSensor);
        Logger.logBool(ClimberConstants.subsystemName, "canSeeTower", canSeeTower());
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean motorConnected = motor.isConnected();
        boolean towerSensorConnected = towerSensor.isConnected();

        CANMonitor.logCANDeviceStatus("climbMotor", ClimberConstants.ClimbMotor.motorID, motorConnected);
        Alerts.climberDisconnected.set(!motorConnected);
        Alerts.climberOverheating.set(motor.getDeviceTemp().getValue().in(Units.Celsius) >= 80);
        Alerts.climberFaults.set(TalonFXUtils.getAllActiveFaults(motor).hasCriticalFaults());

        CANMonitor.logCANDeviceStatus("towerSensor", ClimberConstants.TowerSensor.sensorID, towerSensorConnected);
        Alerts.towerSensorDisconnected.set(!towerSensorConnected);
        Alerts.towerSensorBadStatus.set(MiscUtils.criticalTOFState(towerSensor));
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

    public void dutyCycle(double perc) {
        motorTarget = Optional.empty();

        motor.set(perc);
    }

    public void setAsZero() {
        motor.setPosition(0);
    }

    public ClimberPosition getPosition() {
        return ClimberPosition.fromMotorRotations(motor.getPosition().getValue().in(Units.Rotations));
    }

    public Optional<Boolean> isMotorAtTarget() {
        if (motorTarget.isEmpty()) return Optional.empty();

        double motorRots = getPosition().asMotorRotations();
        double targetRots = motorTarget.get().asMotorRotations();
        double toleranceRots = ClimberConstants.targetTolerance.asMotorRotations();

        return Optional.of(Math.abs(targetRots - motorRots) <= toleranceRots);
    }

    public boolean canSeeTower() {
        return towerSensor.getRange() <= ClimberConstants.towerActivationMM
                && towerSensor.getRangeSigma() <= ClimberConstants.towerActivationStdDev;
    }
}
