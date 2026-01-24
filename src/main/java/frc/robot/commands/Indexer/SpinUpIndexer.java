package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinUpIndexer extends Command {
    public final IndexerSubsystem indexer;

    public SpinUpIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setIndexerVelocity(0);
    }

    @Override
    public boolean isFinished() {
        if (indexer.getVelocityRPM() >= IndexConstants.spindexerMotorVelocity) {
            return true;
        } else {
            return false;
        }
    }
}
