package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntake extends Command {

    private final IntakeSubsystem intake;

    public ReverseIntake(IntakeSubsystem intakeSub) {
        intake = intakeSub;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setRollerVoltage(IntakeConstants.Roller.reverseVoltage);
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
