package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TurretConstants.Motor;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;

public class CalibrateTurret extends Command {

    private final TurretSubsystem turret;
    private Timer endConditionDelay;
    private Timer expirationTimer;

    public CalibrateTurret(TurretSubsystem turretSub) {
        turret = turretSub;
        addRequirements(turretSub);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        turret.dutyCycle(Motor.calibrationSpeed);
        endConditionDelay = new Timer();
        endConditionDelay.start();

        turret.setIsCalibrated(false);
        turret.logTargetPoint(Optional.empty());
        turret.logTargetPredictedPoint(Optional.empty());
        turret.logPredictedOffset(Optional.empty());

        expirationTimer = new Timer();
        expirationTimer.start();
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopTurret();

        if (!interrupted) {
            turret.setEncoderPosition(Motor.calibrationEndPos);
            turret.setIsCalibrated(true);
        }
    }

    @Override
    public boolean isFinished() {
        return endConditionDelay.hasElapsed(Motor.calibrationEndDelay) && turret.isAtCalibrationLimit()
                || expirationTimer.hasElapsed(5);
    }
}
