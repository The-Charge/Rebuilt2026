package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class ShootTurret extends Command {
    public TurretSubsystem turretSubsystem;

    public ShootTurret(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
