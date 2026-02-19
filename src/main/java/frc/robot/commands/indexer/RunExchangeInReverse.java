package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class RunExchangeInReverse extends Command {

    private final IndexerSubsystem indexer;

    public RunExchangeInReverse(IndexerSubsystem indexerSub) {
        indexer = indexerSub;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setExchangeMotorVoltage(-6);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setExchangeMotorVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
