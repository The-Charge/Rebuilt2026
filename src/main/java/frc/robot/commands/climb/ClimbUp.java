package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbUp extends Command {

    private final ClimbSubsystem climbSubsystem;
    private final boolean wait;

    public ClimbUp(ClimbSubsystem climb, boolean waitForTarget) {
        climbSubsystem = climb;
        wait = waitForTarget;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setPosition(ClimberConstants.upPosition);
    }

    @Override
    public boolean isFinished() {
        if (!wait) return true;

        return climbSubsystem.isMotorAtTarget().orElse(true);
    }
}
