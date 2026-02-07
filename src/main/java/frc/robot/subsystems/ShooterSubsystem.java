package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.HoodConfig;
import frc.robot.constants.ShooterConstants.ShootConfig;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import frc.robot.utils.SparkUtils;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex shootMotor;
    // private final Servo hood; // this will likely be a motor, not a servo
    private SparkMax hoodMotor;

    public enum HoodPos {
        UP,
        DOWN,
    }

    private HoodPos hoodPos;

    public ShooterSubsystem() {
        shootMotor = new SparkFlex(ShootConfig.ID, MotorType.kBrushless);
        hoodMotor = new SparkMax(HoodConfig.ID, MotorType.kBrushless);

        hoodPos = HoodPos.DOWN;

        configureMotor();
    }

    public void setHoodPos(HoodPos hp) {
        hoodPos = hp;

        Rotation2d angle = hoodPos == HoodPos.UP ? ShooterConstants.upAngle : ShooterConstants.downAngle;

        hoodMotor
                .getClosedLoopController()
                .setSetpoint(angle.getRadians() * HoodConfig.ticksPerRadian, ControlType.kPosition);
    }

    public void shoot(AngularVelocity speed) {
        shootMotor.getClosedLoopController().setSetpoint(speed.abs(RPM), ControlType.kVelocity);
    }

    public void stopShoot() {
        shootMotor.set(0);
    }

    public HoodPos getHoodPos() {
        return hoodPos;
    }

    private void configureMotor() {
        // shoot

        SparkFlexConfig shootConfig = new SparkFlexConfig();

        shootConfig.closedLoop.pid(ShootConfig.p, ShootConfig.i, ShootConfig.d);

        shootConfig.idleMode(ShootConfig.idleMode);
        shootConfig.smartCurrentLimit(ShootConfig.currentLimit);
        shootConfig.inverted(ShootConfig.inverted);

        shootMotor.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // hood

        SparkMaxConfig hoodConfig = new SparkMaxConfig();

        hoodConfig.closedLoop.pid(HoodConfig.p, HoodConfig.i, HoodConfig.d);

        hoodConfig.idleMode(HoodConfig.idleMode);
        hoodConfig.smartCurrentLimit(HoodConfig.currentLimit);
        hoodConfig.inverted(HoodConfig.inverted);

        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

        Logger.logSubsystem(ShooterConstants.subsystemName, this);

        Logger.logSparkMotor(ShooterConstants.subsystemName, "shoot", shootMotor);

        Alerts.shooterDisconnected.set(!SparkUtils.isConnected(shootMotor));
        Alerts.shooterOverheating.set(shootMotor.getMotorTemperature() >= 80);
        Alerts.shooterWarnings.set(SparkUtils.hasCriticalWarnings(shootMotor.getWarnings()));
        Alerts.shooterFaults.set(SparkUtils.hasCriticalFaults(shootMotor.getFaults()));

        Logger.logSparkMotor(ShooterConstants.subsystemName, "hood", hoodMotor);

        Alerts.hoodDisconnected.set(!SparkUtils.isConnected(hoodMotor));
        Alerts.hoodOverheating.set(hoodMotor.getMotorTemperature() >= 80);
        Alerts.hoodWarnings.set(SparkUtils.hasCriticalWarnings(hoodMotor.getWarnings()));
        Alerts.hoodFaults.set(SparkUtils.hasCriticalFaults(hoodMotor.getFaults()));
    }
}
