package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class CalibrateTurret extends Command {
    private final TurretSubsystem turretSub;

    public CalibrateTurret(TurretSubsystem turretSubsystem) {
        turretSub = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSub.calibrate();
    }

    @Override
    public boolean isFinished() {
        return turretSub.isAtLimit();
    }
}
