package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends Command {

    private final ShooterSubsystem shoot;

    public StopShooter(ShooterSubsystem shootSub) {
        shoot = shootSub;

        addRequirements(shoot);
    }

    @Override
    public void initialize() {
        shoot.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
