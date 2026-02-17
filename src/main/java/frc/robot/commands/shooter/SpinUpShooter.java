package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooter extends Command {

    private final ShooterSubsystem indexer;
    private final boolean wait;

    public SpinUpShooter(ShooterSubsystem indexerSub, boolean waitForTarget) {
        indexer = indexerSub; // so we can use indexer here
        wait = waitForTarget;

        addRequirements(indexer); // reserves the table for indexer
    }

    @Override
    public void initialize() {
        indexer.setShooterMotorVelocity(ShooterConstants.shooterVelocity); // in this instance set velocity to ...
    }

    @Override
    public boolean isFinished() {
        if (!wait) return true;

        return indexer.isShooterAtTarget().orElse(true);
    }
}
