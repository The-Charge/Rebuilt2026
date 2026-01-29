package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;

public class ClimbCommand extends Command {
    private final ClimbSubsystem climbSubsystem;

    public ClimbCommand(ClimbSubsystem climb) {
        climbSubsystem = climb;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setClimbMotorPosition(ClimberConstants.Position);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.getPosition() >= 90;
    }
}
