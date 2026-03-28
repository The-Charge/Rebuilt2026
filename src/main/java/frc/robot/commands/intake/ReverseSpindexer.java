package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants.Spindexer;
import frc.robot.subsystems.IndexerSubsystem;

public class ReverseSpindexer extends Command {

    private final IndexerSubsystem indexer;

    private Timer fastTimer;
    private boolean hasSlowedDown;

    public ReverseSpindexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        fastTimer = new Timer();
        fastTimer.start();
        hasSlowedDown = false;

        indexer.setSpindexerVoltage(Spindexer.fastReverseVoltage);
    }

    @Override
    public void execute() {
        if (fastTimer.hasElapsed(0.5) && !hasSlowedDown) {
            hasSlowedDown = true;
            indexer.setSpindexerVoltage(Spindexer.slowReverseVoltage);
        }
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
