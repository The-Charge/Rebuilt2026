package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;

public class AutoWaitForReadyToShoot extends Command {

    private final TurretSubsystem turret;
    private final Optional<Time> expiration;
    private Optional<Timer> expirationTimer;

    public AutoWaitForReadyToShoot(TurretSubsystem noDepTurretSub, Optional<Time> expirationTime) {
        turret = noDepTurretSub;
        expiration = expirationTime;
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        if (expiration.isPresent()) {
            expirationTimer = Optional.of(new Timer());
            expirationTimer.get().start();
        }
    }

    @Override
    public boolean isFinished() {
        return expirationTimer.map((val) -> val.hasElapsed(expiration.get())).orElse(false)
                || RobotContainer.getInstance().isReadyToShoot()
                        && turret.getIsCalibrated()
                        && turret.isAtTarget().orElse(false);
    }
}
