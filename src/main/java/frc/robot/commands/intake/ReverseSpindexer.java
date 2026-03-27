package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants.Spindexer;
import frc.robot.subsystems.IndexerSubsystem;

public class ReverseSpindexer extends Command {

    private final IndexerSubsystem indexer;

    public ReverseSpindexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setSpindexerVoltage(Spindexer.reverseVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopSpindexer();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
