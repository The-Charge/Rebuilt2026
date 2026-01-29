package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class StartGateIndexer extends Command {

    private final IndexerSubsystem indexer;

    public StartGateIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setGateMotorVoltage(IndexerConstants.gateToShooterVoltage);
    }

    @Override
    public boolean isFinished() { // always returns true since voltage is instant and doesn't need to be checked
        return true;
    }
}
