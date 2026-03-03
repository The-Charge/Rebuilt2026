package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class ManualTurret extends Command {

    private final TurretSubsystem turret;
    private final double duty;

    public ManualTurret(TurretSubsystem turretSub, double dutyCycle) {
        turret = turretSub;
        duty = dutyCycle;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.dutyCycle(duty);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
