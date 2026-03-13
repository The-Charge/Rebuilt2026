package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;

// Calibrates Turret by sending it to hard stop, pausing for a small amount of time, and setting position as zero
public class CalibrateTurret extends Command {

    private final TurretSubsystem turretSub;
    private Timer endConditionDelay;
    private Timer expirationTimer;

    public CalibrateTurret(TurretSubsystem turretSubsystem) {
        turretSub = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    // Start command by starting the turret at a slow speed towards the end position
    @Override
    public void initialize() {
        turretSub.dutyCycle(TurretConstants.calibrationSpeed);
        endConditionDelay = new Timer();
        endConditionDelay.start();

        turretSub.setIsCalibrated(false);
        turretSub.logTargetPoint(Optional.empty());

        expirationTimer = new Timer();
        expirationTimer.start();
    }

    @Override
    public void execute() {}

    // Once timer finishes, the command ends
    @Override
    public boolean isFinished() {
        return endConditionDelay.hasElapsed(TurretConstants.calibrationEndDelay) && turretSub.isAtReverseLimit()
                || expirationTimer.hasElapsed(5);
    }

    // Once command ends, set as end position.
    @Override
    public void end(boolean interrupted) {
        turretSub.stop();
        if (!interrupted) {
            turretSub.setEncoderPosition(TurretConstants.calibrationEndPos);
            turretSub.setIsCalibrated(true);
        }
    }
}
