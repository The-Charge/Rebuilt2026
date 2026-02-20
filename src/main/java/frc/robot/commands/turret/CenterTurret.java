package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class CenterTurret extends Command {

    private final TurretSubsystem turret;

    public CenterTurret(TurretSubsystem turretSub) {
        turret = turretSub;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setTurretAngle(TurretConstants.IllegalAngle.center);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
