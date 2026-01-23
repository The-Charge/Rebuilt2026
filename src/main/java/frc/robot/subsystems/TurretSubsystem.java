package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;

// shoot angle, turret
public class TurretSubsystem extends SubsystemBase{
    private final SparkMax turret;
    private final Servo hood;

    private Rotation2d offset;

    public TurretSubsystem() {
        turret = new SparkMax(TurretConstants.turretId, MotorType.kBrushless);
        hood = new Servo(TurretConstants.hoodChannel);

        configureMotors();
    }

    @Override
    public void periodic() {
    }

    public void setTurretAngle(Rotation2d angle) {
        turret.getClosedLoopController().setSetpoint(angle.plus(offset).getRadians()*TurretConstants.ticksPerRadian, ControlType.kPosition);
    }

    public void setHoodAngle(Rotation2d angle) {
        // IDK some map from angle to servo position
    }

    public void setHoodPos(double val) {
        hood.set(val);
    }

    public void stop() {
        turret.stopMotor();
        // stop the hood servo too
    }

    public Rotation2d getTurretAngle() {
        return new Rotation2d(turret.getEncoder().getPosition() * TurretConstants.radiansPerTick).plus(offset);
    }

    public Rotation2d getHoodAngle() {
        // the reverse of the previos map
        return null;
    }

    public double getHoodPos() {
        return hood.get();
    }

    private void configureMotors() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        config.idleMode(ShooterConstants.idleMode);
        config.smartCurrentLimit(ShooterConstants.currentLimit);
        config.inverted(ShooterConstants.inverted);

        turret.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
