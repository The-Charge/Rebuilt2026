package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunRoller extends Command {
    private IntakeSubsystem intake;

    public RunRoller(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.startRoller(IntakeConstants.Roller.speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
