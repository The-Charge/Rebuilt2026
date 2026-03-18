package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunRoller extends Command {

    private final IntakeSubsystem intake;
    private final boolean instant;

    public RunRoller(IntakeSubsystem intake, boolean isInstant) {
        this.intake = intake;
        this.instant = isInstant;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setRollerVoltage(IntakeConstants.Roller.intakeVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        if (!instant) {
            intake.stopRoller();
        }
    }

    @Override
    public boolean isFinished() {
        return instant;
    }
}
