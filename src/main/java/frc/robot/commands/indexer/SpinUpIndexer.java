package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinUpIndexer extends Command {

    private final IndexerSubsystem indexer;
    private Timer startTimer;

    public SpinUpIndexer(IndexerSubsystem indexerSub) {
        indexer = indexerSub; // so we can use indexer here

        addRequirements(indexer); // reserves the table for indexer
    }

    @Override
    public void initialize() {
        indexer.setExchangeMotorVoltage(IndexerConstants.exchangeVoltage);

        startTimer = new Timer();
        startTimer.start();
    }

    @Override
    public void execute() {
        indexer.setSpindexerVoltage(IndexerConstants.spindexerVoltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
