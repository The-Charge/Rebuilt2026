package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    private final SparkMax turret;
    private Rotation2d offset;

    public TurretSubsystem() {
        turret = new SparkMax(TurretConstants.turretId, MotorType.kBrushless);
        offset = new Rotation2d();
        configureMotor();
    }

    @Override
    public void periodic() {}

    public void setTurretAngle(Rotation2d angle) {
        turret.getClosedLoopController()
                .setSetpoint(angle.plus(offset).getRadians() * TurretConstants.ticksPerRadian, ControlType.kPosition);
    }

    public void stop() {
        turret.set(0);
    }

    public Rotation2d getTurretAngle() {
        return new Rotation2d(turret.getEncoder().getPosition() * TurretConstants.radiansPerTick).plus(offset);
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        config.idleMode(TurretConstants.idleMode);
        config.smartCurrentLimit(TurretConstants.currentLimit);
        config.inverted(TurretConstants.inverted);

        turret.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}