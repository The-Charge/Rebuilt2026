package frc.robot.commands.turret;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FieldVisionAlign extends Command {
    private final SwerveSubsystem swerve;
    private final TurretSubsystem turret;
    private final Pose2d pose;
    private final LimelightSubsystem lsub;
    private boolean isRed;

    public FieldVisionAlign(TurretSubsystem _turret, SwerveSubsystem _swerve, LimelightSubsystem lsub, Pose2d _pose) {
        swerve = _swerve;
        turret = _turret;
        pose = _pose;
        this.lsub = lsub;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Safe alliance getting

        Optional<Alliance> sideOpt = DriverStation.getAlliance();
        
        if (sideOpt.isEmpty()) System.out.println("error no driverstation side");
        isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
    }

    @Override
    public void execute() {
        // Gets Robot pose field relative, subtracts from HubPos to get Robot to Hub vector (vectorDifference), 
        // then sets the turret angle to robot rotation + angle.
        Optional<Pose2d> robotPoseOptional = lsub.getVisionMeasurementMegaTag1();
        if (robotPoseOptional.isEmpty()) return;
        Pose2d robotPose = robotPoseOptional.get();
        Transform2d vectorDifference = (isRed ? FieldConstants.redHubPos : FieldConstants.blueHubPos).minus(robotPose);
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
