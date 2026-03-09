package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.Supplier;

public class ManualShoot extends Command {

    private final ShooterSubsystem shoot;
    private final Supplier<AngularVelocity> targetVel;

    public ManualShoot(ShooterSubsystem shootSub, Supplier<AngularVelocity> vel) {
        shoot = shootSub;
        targetVel = vel;

        addRequirements(shoot);
    }

    @Override
    public void execute() {
        shoot.shoot(targetVel.get());
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
