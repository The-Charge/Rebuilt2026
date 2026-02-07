package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;
import java.util.Optional;

// shoot angle, turret
public class TurretSubsystem extends SubsystemBase {
    private final SparkMax spin;
    // private final SparkFlex shooter;
    // private final Servo hood;

    private Rotation2d offset = new Rotation2d();
    private Optional<Rotation2d> targetAngle;

    public TurretSubsystem() {
        spin = new SparkMax(TurretConstants.Spin.motorID, MotorType.kBrushless); // port number under IndexerConstants
        SparkMaxConfig spindexerConfig = new SparkMaxConfig();
        SparkUtils.configureBasicSettings(
                spindexerConfig,
                TurretConstants.Spin.maxCurrent,
                TurretConstants.Spin.idleMode,
                TurretConstants.Spin.inverted,
                TurretConstants.Spin.maxDutyCycle,
                TurretConstants.Spin.nominalVoltage);
        SparkUtils.configureClosedLoopSettings(
                spindexerConfig,
                TurretConstants.Spin.kP,
                TurretConstants.Spin.kI,
                TurretConstants.Spin.kD,
                TurretConstants.Spin.kStaticG,
                TurretConstants.Spin.kCos,
                TurretConstants.Spin.kS,
                TurretConstants.Spin.kV,
                TurretConstants.Spin.kA);
        if (spin.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
                != REVLibError.kOk) {
            Logger.reportError("Failed to configure spindexer motor");
            // Alerts.spindexerConfigFail.set(true);
        }

        targetAngle = Optional.empty();

        // shooter = new SparkFlex(TurretConstants.shooterId, MotorType.kBrushless);
        // hood = new Servo(TurretConstants.hoodChannel);
    }

    @Override
    public void periodic() {
        Logger.logSparkMotor("Turret", "angleing", spin);
    }

    public void setTurretAngle(Rotation2d angle) {
        if (angle == null) {
            Logger.reportWarning("Cannot set spindexer velocity to a null velocity", true);
            return;
        }

        targetAngle = Optional.of(angle);

        double request = angle.getRadians();
        spin.getClosedLoopController()
                .setSetpoint(request, ControlType.kPosition);
    }

    public void setHoodAngle(Rotation2d angle) {
        // IDK some map from angle to servo position
    }

    public void setHoodPos(double val) {
        // hood.set(val);
    }

    public void shoot(double speed) {
        // shooter.set(speed); // dumb
    }

    public void stop() {
        spin.set(0);
        // shooter.set(0);
        // stop the hood servo too
    }

    public Rotation2d getTurretAngle() {
        return new Rotation2d(spin.getEncoder().getPosition() * TurretConstants.Spin.radiansPerTick).plus(offset);
    }

    public Rotation2d getHoodAngle() {
        // the reverse of the previos map
        return null;
    }

    // public double getHoodPos() {
    //     return hood.get();
    // }
}
