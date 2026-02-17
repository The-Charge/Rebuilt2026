package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinDownShooter extends Command {

    private final ShooterSubsystem shooter;

    public SpinDownShooter(ShooterSubsystem shooterSub) {
        shooter = shooterSub;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stopAll();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
