package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class StopGateIndexer extends Command {

    private final IndexerSubsystem indexer;

    public StopGateIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setGateMotorVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
