package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexer extends Command {

    private final IndexerSubsystem indexer;
    private final boolean instant;

    public RunIndexer(IndexerSubsystem indexerSub, boolean isInstant) {
        indexer = indexerSub;
        instant = isInstant;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setExchangeMotorVoltage(IndexerConstants.exchangeVoltage);
        indexer.setSpindexerVoltage(IndexerConstants.spindexerVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        if (!instant) {
            indexer.stopAll();
        }
    }

    @Override
    public boolean isFinished() {
        return instant;
    }
}
