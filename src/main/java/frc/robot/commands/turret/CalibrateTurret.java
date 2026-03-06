package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;

public class CalibrateTurret extends Command {

    private final TurretSubsystem turretSub;
    private Optional<Timer> zeroingDelay;

    public CalibrateTurret(TurretSubsystem turretSubsystem) {
        turretSub = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSub.dutyCycle(TurretConstants.calibrationSpeed);
        zeroingDelay = Optional.empty();
        turretSub.setIsCalibrated(false);
    }

    @Override
    public void execute() {
        if (turretSub.isAtForwardLimit() && zeroingDelay.isEmpty()) {
            zeroingDelay = Optional.of(new Timer());
            zeroingDelay.get().start();

            turretSub.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) turretSub.setEncoderPosition(TurretConstants.calibrationEndPos);
        turretSub.setIsCalibrated(true);
    }

    @Override
    public boolean isFinished() {
        return zeroingDelay.isPresent() && zeroingDelay.get().hasElapsed(TurretConstants.calibrationResetDelay);
    }
}
