package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;

public class AlignTurret extends Command {
    private final SwerveSubsystem swerveSub;
    private final TurretSubsystem turretSub;
    private final LimelightSubsystem limelightSub;

    private final Optional<Boolean> isRed;
    private final Optional<Pose2d> targetPose;

    public AlignTurret(
            TurretSubsystem turret, SwerveSubsystem swerve, LimelightSubsystem limelight, Pose2d targetPose) {

        this.swerveSub = swerve;
        this.turretSub = turret;
        this.limelightSub = limelight;

        this.targetPose = Optional.of(targetPose);
        this.isRed = Optional.empty();

        addRequirements(turret);
    }

    // Align the turret to a specific hub if we can see the tags
    public AlignTurret(
            TurretSubsystem turret, SwerveSubsystem swerve, LimelightSubsystem limelight, Alliance alliance) {

        this.swerveSub = swerve;
        this.turretSub = turret;
        this.limelightSub = limelight;

        this.isRed = Optional.of(alliance == Alliance.Red);
        this.targetPose = Optional.empty();

        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (isRed.isPresent()) {
            boolean succeeded = hubTagAlign(isRed.get());

            if (!succeeded) {
                swerveAlign(FieldConstants.getHubPos(isRed.get()));
            }
            return;
        }

        // TODO: if hub failed, get the hub pose manually

        if (targetPose.isPresent()) {
            swerveAlign(targetPose.get());
        }
    }

    //
    public boolean hubTagAlign(boolean isRed) {
        // Get Detection (safe)
        // change based on which alliance
        Optional<Pose3d> poseOpt =
                limelightSub.getTransformToTag(FieldConstants.getHubTag(isRed));

        // Gets actual pose (safe)
        if (poseOpt.isEmpty()) return false; // TODO: log that this failed
        Pose3d pose = poseOpt.get();

        // Set turret angle to robotToHub vector
        Rotation2d rotationToHub = new Rotation2d(pose.getX(), pose.getY());

        SmartDashboard.putNumber("rotation given to turret", rotationToHub.getRadians());
        turretSub.setTurretAngle(rotationToHub.plus(new Rotation2d(Math.PI)));

        return true;
    }

    public void swerveAlign(Pose2d targetPose) {
        Pose2d robotPose = swerveSub.getPose();
        Transform2d vectorDifference = targetPose.minus(robotPose);
        double angleFieldRelative = Math.atan2(vectorDifference.getY(), vectorDifference.getX());
        turretSub.setTurretAngle(new Rotation2d(angleFieldRelative).plus(robotPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
