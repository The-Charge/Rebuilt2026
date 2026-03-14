package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoStopShoot extends Command {

    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;

    public AutoStopShoot(ShooterSubsystem shooterSub, IndexerSubsystem indexerSub, IntakeSubsystem intakeSub) {
        shooter = shooterSub;
        indexer = indexerSub;
        intake = intakeSub;

        // TODO: require subsystems
    }

    @Override
    public void initialize() {
        shooter.stopShoot();
        indexer.stopAll();
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
