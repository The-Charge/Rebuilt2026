package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class StopIndexer extends Command {

    private final IndexerSubsystem indexer;

    public StopIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
