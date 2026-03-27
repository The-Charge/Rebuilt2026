package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;

public class ManualTurret extends Command {

    private final TurretSubsystem turret;
    private final double duty;

    public ManualTurret(TurretSubsystem turretSub, double dutyCycle) {
        turret = turretSub;
        duty = dutyCycle;

        addRequirements(turret);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        turret.dutyCycle(duty);
        turret.logTargetPoint(Optional.empty());
        turret.logTargetPredictedPoint(Optional.empty());
        turret.logPredictedOffset(Optional.empty());
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopTurret();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
