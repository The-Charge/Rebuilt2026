package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StopRoller extends Command {

    private final IntakeSubsystem intake;

    public StopRoller(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
