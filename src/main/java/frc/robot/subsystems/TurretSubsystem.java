package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TurretConstants;

// shoot angle, turret
public class TurretSubsystem extends SubsystemBase{
        private SparkMax turret;
        private Servo hood;
        public TurretSubsystem() {
            turret = new SparkMax(TurretConstants.turretId, MotorType.kBrushless);
            hood = new Servo(TurretConstants.hoodChannel);
            
            configureMotor(turret, false);

    }
    private void configureMotor(SparkMax m, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(ShooterConstants.idleMode);
        config.smartCurrentLimit(ShooterConstants.currentLimit);
        config.inverted(inverted);

        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
