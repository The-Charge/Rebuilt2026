package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinUpIndexer extends Command {
    public final IndexerSubsystem indexer;

    public SpinUpIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub; // so we can use indexer here
        addRequirements(indexer); // reserves the table for indexer
    }

    @Override
    public void initialize() {
        indexer.setSpindexerMotorVelocity(
                IndexConstants.spindexerMotorVelocity); // in this instance set velocity to ...
    }

    @Override
    public boolean isFinished() {
        if (indexer.getSpindexerRPM() >= IndexConstants.spindexerMotorVelocity) {
            return true;
        } else {
            return false;
        } // if velocity has reached or surpassed the set velocity then it will stop.
    }
}
