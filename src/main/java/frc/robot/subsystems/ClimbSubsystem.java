package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ClimberConstants.Motor;
import frc.robot.constants.ClimberConstants.TowerSensor;
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

    private final Alert motorDisconnected, motorOverheating, motorFaults, motorConfigFail;
    private final Alert towerSensorDisconnected, towerSensorBadStatus;

    private Optional<ClimberPosition> motorTarget;

    public ClimbSubsystem() {
        motorDisconnected = Alerts.makeDisconnectAlert(Motor.motorName, Motor.motorID);
        motorOverheating = Alerts.makeOverheatingAlert(Motor.motorName, Motor.motorID);
        motorFaults = Alerts.makeCriticalFaultsAlert(Motor.motorName, Motor.motorID);
        motorConfigFail = Alerts.makeConfigFailAlert(Motor.motorName, Motor.motorID);
        towerSensorDisconnected = Alerts.makeDisconnectAlert("tower sensor", ClimberConstants.TowerSensor.sensorID);
        towerSensorBadStatus = new Alert(
                String.format("Potentially bad status on tower sensor (CAN %d)", ClimberConstants.TowerSensor.sensorID),
                AlertType.kWarning);

        motor = new TalonFX(Motor.motorID);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        TalonFXUtils.configureBasicSettings(
                motorConfig, Motor.maxCurrent, Motor.neutralMode, Motor.inverted, Motor.maxDutyCycle, Motor.maxVoltage);
        TalonFXUtils.configureClosedLoopSettings(
                motorConfig,
                Motor.kP,
                Motor.kI,
                Motor.kD,
                Motor.kG,
                Motor.kGType,
                Motor.kS,
                Motor.kSSign,
                Motor.kV,
                Motor.kA);
        if (motor.getConfigurator().apply(motorConfig) != StatusCode.OK) {
            Logger.reportError(String.format("Failed to configure %s", Motor.motorName));
            motorConfigFail.set(true);
        }

        towerSensor = new TimeOfFlight(TowerSensor.sensorID);
        towerSensor.setRangingMode(TowerSensor.rangingMode, TowerSensor.sampleTime.in(Milliseconds));

        motorTarget = Optional.empty();
    }

    @Override
    public String getName() {
        return ClimberConstants.subsystemName;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(getName(), this);

        Logger.logTalonFX(getName(), Motor.motorName, motor);
        Logger.logDouble(
                getName(),
                "targetMotorRots",
                motorTarget.map((val) -> val.asMotorRotations()).orElse(Double.NaN));
        Logger.logBool(getName(), "isAtTarget", isMotorAtTarget().orElse(true));

        Logger.logTOF(getName(), TowerSensor.sensorName, towerSensor);
        Logger.logBool(getName(), "canSeeTower", canSeeTower());
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {
        boolean motorConnected = motor.isConnected();
        boolean towerSensorConnected = towerSensor.isConnected();

        CANMonitor.logCANDeviceStatus(Motor.motorName, Motor.motorID, motorConnected);
        motorDisconnected.set(!motorConnected);
        motorOverheating.set(motor.getDeviceTemp().getValue().in(Units.Celsius) >= 80);
        motorFaults.set(TalonFXUtils.getAllActiveFaults(motor).hasCriticalFaults());

        CANMonitor.logCANDeviceStatus(TowerSensor.sensorName, TowerSensor.sensorID, towerSensorConnected);
        towerSensorDisconnected.set(!towerSensorConnected);
        towerSensorBadStatus.set(MiscUtils.criticalTOFState(towerSensor));
    }

    public void setPosition(ClimberPosition position) {
        if (position == null) {
            Logger.reportWarning(String.format("Cannot move %s to a null ClimberPosition", Motor.motorName), true);
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
        double toleranceRots = Motor.targetTolerance.asMotorRotations();

        return Optional.of(Math.abs(targetRots - motorRots) <= toleranceRots);
    }

    public boolean canSeeTower() {
        return towerSensor.getRange() <= TowerSensor.activationMM
                && towerSensor.getRangeSigma() <= TowerSensor.activationStdDev
                && towerSensor.isRangeValid();
    }
}
