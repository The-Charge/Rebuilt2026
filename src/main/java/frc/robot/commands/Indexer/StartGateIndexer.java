package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class StartGateIndexer extends Command {
    public final IndexerSubsystem indexer;

    public StartGateIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setVoltage(IndexConstants.gateToShooterVoltage);
    }

    @Override
    public boolean isFinished() {
        if (indexer.getVoltage() >= IndexConstants.gateToShooterVoltage) {
            return true;
        } else {
            return false;
        }
    }
}
