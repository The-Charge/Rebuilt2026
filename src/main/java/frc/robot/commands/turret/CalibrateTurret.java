package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;

// Calibrates Turret by sending it to hard stop, pausing for a small amount of time, and setting position as zero
public class CalibrateTurret extends Command {

    private final TurretSubsystem turretSub;
    private Optional<Timer> zeroingDelay;

    public CalibrateTurret(TurretSubsystem turretSubsystem) {
        turretSub = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    // Start command by starting the turret at a slow speed towards the end position
    @Override
    public void initialize() {
        turretSub.dutyCycle(TurretConstants.calibrationSpeed);
        zeroingDelay = Optional.empty();
        turretSub.setIsCalibrated(false);
    }

    @Override
    public void execute() {
        // When at hard stop, stop the motor and start the timer
        if (turretSub.isAtForwardLimit() && zeroingDelay.isEmpty()) {
            zeroingDelay = Optional.of(new Timer());
            zeroingDelay.get().start();

            turretSub.stop();
        }
    }

    // Once timer finishes, the command ends
    @Override
    public boolean isFinished() {
        return zeroingDelay.isPresent() && zeroingDelay.get().hasElapsed(TurretConstants.calibrationResetDelay);
    }

    // Once command ends, set as end position.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            turretSub.setEncoderPosition(TurretConstants.calibrationEndPos);
            turretSub.setIsCalibrated(true);
        }
    }

    
}
