package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.units.TurretAngle;
import frc.robot.utils.ControllerUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DemoAiming extends Command {

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final BooleanSupplier overrideTurret;
    private final Supplier<AngularVelocity> shooterSpeed;

    public DemoAiming(
            TurretSubsystem turretSub,
            ShooterSubsystem shooterSub,
            BooleanSupplier turretOverride,
            Supplier<AngularVelocity> shooterSpeed) {
        turret = turretSub;
        shooter = shooterSub;
        addRequirements(turret, shooter);
        overrideTurret = turretOverride;
        this.shooterSpeed = shooterSpeed;
    }

    @Override
    public void execute() {
        if (overrideTurret.getAsBoolean()) {
            turret.dutyCycle(-ControllerUtil.applyLinearDeadband(
                            RobotContainer.getInstance().hidDriver2.getLeftX()
                                    + RobotContainer.getInstance().hidDriver2.getRightX(),
                            SwerveConstants.driveJoystickDeadband)
                    * 0.1);
        } else {
            turret.setTurretAngle(TurretAngle.fromMechanismAngle(Degrees.of(0)));
        }

        if (RobotContainer.getInstance().hidDriver2.getRightBumperButton()) {
            shooter.setTargetVelocity(shooterSpeed.get());
        } else {
            shooter.stopShooter();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopTurret();
        shooter.stopShooter();
    }
}
