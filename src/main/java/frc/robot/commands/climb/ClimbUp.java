package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbUp extends Command {
    private final ClimbSubsystem climbSubsystem;

    public ClimbUp(ClimbSubsystem climb) {
        climbSubsystem = climb;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setClimbMotorPosition(ClimberConstants.UpPosition);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.getPosition().toMotorRots() >= ClimberConstants.UpPosition.toMotorRots();
    }
}
