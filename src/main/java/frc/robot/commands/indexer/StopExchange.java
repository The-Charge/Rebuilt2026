package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class StopExchange extends Command {

    private final IndexerSubsystem indexer;

    public StopExchange(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setExchangeMotorVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
