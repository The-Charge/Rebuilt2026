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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.ShooterConstants.HoodConfig;
import frc.robot.constants.ShooterConstants.ShootConfig;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex shootMotor;
    // private final Servo hood; // this will likely be a motor, not a servo
    private SparkMax hoodMotor;

    private HoodPos hoodPos;

    public ShooterSubsystem() {
        shootMotor = new SparkFlex(ShootConfig.ID, MotorType.kBrushless);
        hoodMotor = new SparkMax(HoodConfig.ID, MotorType.kBrushless);

        hoodPos = HoodPos.DOWN;

        configureMotor();
    }

    @Override
    public void periodic() {}

    public void setHoodPos(HoodPos hp) {

        hoodPos = hp;
        // move the hood with whatever actuator
        if (hp == HoodPos.UP)
            hoodMotor
                    .getClosedLoopController()
                    .setSetpoint(
                            ShooterConstants.upAngle.getRadians() * HoodConfig.ticksPerRadian, ControlType.kPosition);
        else
            hoodMotor
                    .getClosedLoopController()
                    .setSetpoint(
                            ShooterConstants.downAngle.getRadians() * HoodConfig.ticksPerRadian, ControlType.kPosition);
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

    public enum HoodPos {
        UP,
        DOWN,
    }
}
