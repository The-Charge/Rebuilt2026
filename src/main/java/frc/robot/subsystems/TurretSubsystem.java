package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;

// shoot angle, turret
public class TurretSubsystem extends SubsystemBase {
    private final SparkMax spin;
    private final SparkFlex shooter;
    // private final Servo hood; // this will likely be a motor, not a servo

    private boolean isHoodUp;

    private Rotation2d offset;

    public TurretSubsystem() {
        spin = new SparkMax(TurretConstants.turretId, MotorType.kBrushless);
        shooter = new SparkFlex(TurretConstants.shooterId, MotorType.kBrushless);
        // hood = new Servo(TurretConstants.hoodChannel);

        isHoodUp = false;

        configureMotors();
    }

    @Override
    public void periodic() {}

    public void setTurretAngle(Rotation2d angle) {
        spin.getClosedLoopController()
                .setSetpoint(angle.plus(offset).getRadians() * TurretConstants.ticksPerRadian, ControlType.kPosition);
    }

    public void setHoodAngle(boolean isUp) {
        isHoodUp = isUp;
        // move the hood with whatever actuator
    }

    public void shoot(double speed) {
        shooter.set(speed); // dumb
    }

    public void stop() {
        spin.set(0);
        shooter.set(0);
        // stop the hood motor too
    }

    public Rotation2d getTurretAngle() {
        return new Rotation2d(spin.getEncoder().getPosition() * TurretConstants.radiansPerTick).plus(offset);
    }

    private void configureMotors() { // BAD. Constants reused and servo not configured
        SparkMaxConfig maxconfig = new SparkMaxConfig();
        maxconfig.closedLoop.pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        maxconfig.idleMode(TurretConstants.idleMode);
        maxconfig.smartCurrentLimit(TurretConstants.currentLimit);
        maxconfig.inverted(TurretConstants.inverted);

        spin.configure(maxconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig flexconfig = new SparkFlexConfig();
        flexconfig.idleMode(TurretConstants.idleMode);
        flexconfig.smartCurrentLimit(TurretConstants.currentLimit);
        flexconfig.inverted(TurretConstants.inverted);

        shooter.configure(flexconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
