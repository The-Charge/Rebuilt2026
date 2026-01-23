package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
    private IntakeSubsystem intake;

    public DeployIntake(IntakeSubsystem intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.deploy();
    }

    @Override
    public void end(boolean interrupted) {
        intake.startRoller();
    }

    @Override
    public boolean isFinished() {
        return intake.isDeployed();
    }
}
