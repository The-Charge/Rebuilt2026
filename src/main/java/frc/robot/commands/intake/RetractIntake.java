package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntake extends Command {
    private IntakeSubsystem intake;

    public RetractIntake(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stopRoller();
        intake.retract();
    }

    @Override
    public void end(boolean interrupted) {
        // idk
    }

    @Override
    public boolean isFinished() {
        return intake.isRetracted();
    }
}
