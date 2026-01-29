package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinUpIndexer extends Command {

    private final IndexerSubsystem indexer;
    private final boolean wait;

    public SpinUpIndexer(IndexerSubsystem indexerSub, boolean waitForTarget) {
        indexer = indexerSub; // so we can use indexer here
        wait = waitForTarget;

        addRequirements(indexer); // reserves the table for indexer
    }

    @Override
    public void initialize() {
        indexer.setSpindexerMotorVelocity(
                IndexerConstants.spindexerMotorVelocity); // in this instance set velocity to ...
    }

    @Override
    public boolean isFinished() {
        if (!wait) return true;

        return indexer.isSpindexerAtTarget().orElse(true);
    }
}
