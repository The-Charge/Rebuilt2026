package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IndexerConstants.Exchange;
import frc.robot.constants.IndexerConstants.Spindexer;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoShoot extends Command {

    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    public AutoShoot(IndexerSubsystem indexerSub, IntakeSubsystem intakeSub) {
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
        indexer.setExchangeMotorVoltage(Exchange.shootVoltage);
        indexer.setSpindexerVoltage(Spindexer.shootVoltage);
        intake.setRollerVoltage(IntakeConstants.Roller.shootVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stopAll();
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
