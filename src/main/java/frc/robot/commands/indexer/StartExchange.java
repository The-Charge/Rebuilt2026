package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class StartExchange extends Command {

    private final IndexerSubsystem indexer;

    public StartExchange(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setExchangeMotorVoltage(IndexerConstants.exchangeVoltage);
    }

    @Override
    public boolean isFinished() { // always returns true since voltage is instant and doesn't need to be checked
        return true;
    }
}
