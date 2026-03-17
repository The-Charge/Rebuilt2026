package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class PreSpinShooter extends Command {

    private final ShooterSubsystem shooter;

    public PreSpinShooter(ShooterSubsystem shooterSub) {
        shooter = shooterSub;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetVelocity(ShooterConstants.preSpinSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
