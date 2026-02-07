package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex shoot;
    // private final Servo hood; // this will likely be a motor, not a servo
    private SparkMax hood;

    private HoodPos hoodPos;

    public ShooterSubsystem() {
        shoot = new SparkFlex(ShooterConstants.shooterId, MotorType.kBrushless);
        hood = new SparkMax(ShooterConstants.hoodId, MotorType.kBrushless);

        hoodPos = HoodPos.DOWN;

        configureMotor();
    }

    @Override
    public void periodic() {}

    public void setHoodPos(HoodPos hp) {

        hoodPos = hp;
        // move the hood with whatever actuator
        if (hp == HoodPos.UP)
            hood.getClosedLoopController()
                    .setSetpoint(
                            ShooterConstants.upAngle.getRadians() * ShooterConstants.hoodTicksPerRadian,
                            ControlType.kPosition);
        else
            hood.getClosedLoopController()
                    .setSetpoint(
                            ShooterConstants.downAngle.getRadians() * ShooterConstants.hoodTicksPerRadian,
                            ControlType.kPosition);
    }

    public void shoot() {
        shoot.set(ShooterConstants.shootSpeed);
    }

    public void stopShoot() {
        shoot.set(0);
    }

    public HoodPos getHoodPos() {
        return hoodPos;
    }

    private void configureMotor() {
        // shoot
        SparkFlexConfig config1 = new SparkFlexConfig();
        config1.idleMode(ShooterConstants.idleMode);
        config1.smartCurrentLimit(ShooterConstants.currentLimit);
        config1.inverted(ShooterConstants.shootInverted);

        shoot.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // hood

        SparkMaxConfig config2 = new SparkMaxConfig();

        config2.closedLoop.pid(ShooterConstants.shootkP, ShooterConstants.shootkI, ShooterConstants.shootkD);

        config2.idleMode(ShooterConstants.idleMode);
        config2.smartCurrentLimit(ShooterConstants.currentLimit);
        config2.inverted(ShooterConstants.hoodInverted);

        hood.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public enum HoodPos {
        UP,
        DOWN
    }
}
