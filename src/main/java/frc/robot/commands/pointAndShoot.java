package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class pointAndShoot extends Command {
    private final TurretSubsystem turretSub;

    public pointAndShoot(TurretSubsystem tsub) {
        turretSub = tsub;

        addRequirements(turretSub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Rotation2d angle = new Rotation2d(); // get the rotation from the limelight
        turretSub.setTurretAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {
        turretSub.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
