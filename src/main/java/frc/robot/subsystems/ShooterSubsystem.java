package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;


// , shooter
public class ShooterSubsystem extends SubsystemBase {
    private SparkFlex shooter;

    public ShooterSubsystem() {
        shooter = new SparkFlex(ShooterConstants.shooterId, MotorType.kBrushless);
        configureMotor(shooter, false); // change inverted
    }

    public void stop() {
        shooter.set(0);
    }

    // maybe change? new SparkFlex
    private void configureMotor(SparkFlex m, boolean inverted) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(ShooterConstants.idleMode);
        config.smartCurrentLimit(ShooterConstants.currentLimit);
        config.inverted(inverted);

        m.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}