package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.ControllerUtil;

public class JoystickTurret extends Command {

    private final TurretSubsystem turret;

    public JoystickTurret(TurretSubsystem turretSub) {
        turret = turretSub;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.dutyCycle(-ControllerUtil.applyLinearDeadband(
                        RobotContainer.getInstance().hidDriver2.getLeftX(), SwerveConstants.driveJoystickDeadband)
                * 0.1);
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
