package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class PointAtPose extends Command {
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    private final Pose2d pose;

    public PointAtPose(TurretSubsystem _turret, SwerveSubsystem _swerve, Pose2d _pose) {
        swerve = _swerve;
        turret = _turret;
        pose = _pose;
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        Transform2d vectorDifference = pose.minus(robotPose);
        double angleFieldRelative = Math.atan2(vectorDifference.getY(), vectorDifference.getX());
        turret.setTurretAngle(new Rotation2d(angleFieldRelative).plus(robotPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
