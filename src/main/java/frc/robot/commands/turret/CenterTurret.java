package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.units.TurretAngle;

public class CenterTurret extends Command {

    private final TurretSubsystem turret;

    public CenterTurret(TurretSubsystem turretSub) {
        turret = turretSub;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setTurretAngle(TurretAngle.fromMechanismRotations(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
