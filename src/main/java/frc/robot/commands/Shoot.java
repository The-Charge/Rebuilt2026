package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants.Exchange;
import frc.robot.constants.IndexerConstants.Spindexer;
import frc.robot.constants.IntakeConstants.Roller;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Shoot extends Command {

    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final boolean instant;

    public Shoot(IndexerSubsystem indexerSub, IntakeSubsystem intakeSub, boolean isInstant) {
        indexer = indexerSub;
        intake = intakeSub;
        instant = isInstant;

        addRequirements(indexer);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        indexer.setExchangeMotorVoltage(Exchange.shootVoltage);
        indexer.setSpindexerVoltage(Spindexer.shootVoltage);
    }

    @Override
    public void execute() {
        intake.setRollerVoltage(Roller.shootVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        if (!instant) {
            indexer.stopAll();
            intake.stopRoller();
        }
    }

    @Override
    public boolean isFinished() {
        return instant;
    }
}
