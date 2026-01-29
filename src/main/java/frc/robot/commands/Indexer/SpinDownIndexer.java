package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinDownIndexer extends Command {
    public final IndexerSubsystem indexer;

    public SpinDownIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setSpindexerMotorVelocity(0); // in this instance set velocity to ...
    }

    @Override
    public boolean isFinished() {
        if (indexer.getSpindexerRPM() > 0) {
            return false;
        } else {
            return true;
        } // if velocity has reached or surpassed the set velocity then it will stop.
    }
}
