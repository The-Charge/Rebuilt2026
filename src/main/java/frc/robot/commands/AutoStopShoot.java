package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoStopShoot extends Command {

    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;

    public AutoStopShoot(ShooterSubsystem shooterSub, IndexerSubsystem indexerSub) {
        shooter = shooterSub;
        indexer = indexerSub;
    }

    @Override
    public void initialize() {
        shooter.stopShoot();
        ;
        indexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
