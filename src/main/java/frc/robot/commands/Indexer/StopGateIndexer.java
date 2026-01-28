package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class StopGateIndexer extends Command {
    public final IndexerSubsystem indexer;

    public StopGateIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        if (indexer.getVoltage() > 0) {
            return false;
        } else {
            return true;
        }
    }
}
