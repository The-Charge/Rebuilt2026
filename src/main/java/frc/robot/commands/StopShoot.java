package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StopShoot extends Command {

    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    public StopShoot(IndexerSubsystem indexerSub, IntakeSubsystem intakeSub) {
        indexer = indexerSub;
        intake = intakeSub;
        addRequirements(indexer, intake);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        indexer.stopAll();
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
