package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinDownIndexer extends Command {

    private final IndexerSubsystem indexer;

    public SpinDownIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.stopSpindexer();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
