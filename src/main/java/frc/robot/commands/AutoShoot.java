package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoShoot extends Command {

    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    public AutoShoot(IndexerSubsystem indexerSub, IntakeSubsystem intakeSub) {
        indexer = indexerSub;
        intake = intakeSub;

        addRequirements(indexer); //TODO: fix
    }

    @Override
    public void initialize() {
        indexer.setExchangeMotorVoltage(IndexerConstants.exchangeVoltage);
        indexer.setSpindexerVoltage(IndexerConstants.spindexerVoltage);
        intake.setRollerVoltage(IntakeConstants.Roller.intakeVoltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
