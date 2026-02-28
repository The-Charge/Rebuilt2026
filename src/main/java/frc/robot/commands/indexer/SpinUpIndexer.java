package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinUpIndexer extends Command {

    private final IndexerSubsystem indexer;

    public SpinUpIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub; // so we can use indexer here

        addRequirements(indexer); // reserves the table for indexer
    }

    @Override
    public void initialize() {
        indexer.setSpindexerVoltage(IndexerConstants.spindexerVoltage);
        indexer.setExchangeMotorVoltage(IndexerConstants.exchangeVoltage);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
