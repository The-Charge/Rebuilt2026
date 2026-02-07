package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax turretMotor;
    private Rotation2d offset;

    public TurretSubsystem() {
        turretMotor = new SparkMax(TurretConstants.turretId, MotorType.kBrushless);
        offset = new Rotation2d();
        configureMotor();
    }

    public void setTurretAngle(Rotation2d angle) {
        turretMotor
                .getClosedLoopController()
                .setSetpoint(angle.plus(offset).getRadians() * TurretConstants.ticksPerRadian, ControlType.kPosition);
    }

    public void stop() {
        turretMotor.set(0);
    }

    public Rotation2d getTurretAngle() {
        return new Rotation2d(turretMotor.getEncoder().getPosition() * TurretConstants.radiansPerTick).plus(offset);
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        config.idleMode(TurretConstants.idleMode);
        config.smartCurrentLimit(TurretConstants.currentLimit);
        config.inverted(TurretConstants.inverted);

        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(TurretConstants.subsystemName, this);

        Logger.logSparkMotor(TurretConstants.subsystemName, "motor", turretMotor);

        Alerts.turretDisconnected.set(!SparkUtils.isConnected(turretMotor));
        Alerts.turretOverheating.set(turretMotor.getMotorTemperature() >= 80);
        Alerts.turretWarnings.set(SparkUtils.hasCriticalWarnings(turretMotor.getWarnings()));
        Alerts.turretFaults.set(SparkUtils.hasCriticalFaults(turretMotor.getFaults()));
    }
}
