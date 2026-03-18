package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class ManualShoot extends Command {

    private final ShooterSubsystem shoot;
    private final Supplier<AngularVelocity> targetVel;

    private final String key = String.format("%s/speed", getName());

    public ManualShoot(ShooterSubsystem shootSub, Supplier<AngularVelocity> vel) {
        shoot = shootSub;
        targetVel = vel;

        addRequirements(shoot);
    }

    @Override
    public void initialize() {
        if (!SmartDashboard.containsKey(key)) {
            SmartDashboard.putNumber(key, 0);
        }
    }

    @Override
    public void execute() {
        if (ShooterConstants.manualShootUseSmartdashboard) {
            shoot.setTargetVelocity(RPM.of(SmartDashboard.getNumber(key, 0)));
        } else {
            shoot.setTargetVelocity(targetVel.get());
        }
    }

    @Override
    public void end(boolean interrupted) {
        // shoot.stopShoot();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
