package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;

public class AlignTurret extends Command {
    // private final SwerveSubsystem swerveSub;
    private final TurretSubsystem turretSub;
    // private final LimelightSubsystem limelightSub;

    private final Optional<Boolean> isRed;
    private final Optional<Pose2d> targetPose;

    // Use AlignTurret to align Turret to a specified Pose (usually given by Swerve, or LimelightHelpers.getBotPose())
    public AlignTurret(TurretSubsystem turret, Pose2d targetPose) {

        this.turretSub = turret;

        this.targetPose = Optional.of(targetPose);
        this.isRed = Optional.empty();

        addRequirements(turret);
    }

    // Align the turret directly to AprilTag on Hub (given by Alliance)
    public AlignTurret(TurretSubsystem turret, Alliance alliance) {

        this.turretSub = turret;

        this.isRed = Optional.of(alliance == Alliance.Red);
        this.targetPose = Optional.empty();

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // SmartDashboard.putNumber("turretAngleDeg", 60);
    }

    @Override
    public void execute() {
        // If using Hub tag
        if (isRed.isPresent()) {
            boolean succeeded = hubTagAlign(isRed.get());

            // If can't see hub tag, use swervePose
            if (!succeeded) {
                swerveAlign(FieldConstants.getHubPos(isRed.get()));
            }
            return;
        }

        // If supplied given pose
        if (targetPose.isPresent()) {
            swerveAlign(targetPose.get());
        }
    }

    //
    public boolean hubTagAlign(boolean isRed) {
        // Get Detection (safe)
        // change based on which alliance
        // Optional<Pose3d> poseOpt = limelightSub.getTransformToTag(FieldConstants.getHubTag(isRed));
        Optional<Pose3d> poseOpt = Optional.empty();

        // Gets actual pose (safe)
        if (poseOpt.isEmpty()) return false; // TODO: log that this failed
        Pose3d pose = poseOpt.get();

        // Set turret angle to robotToHub vector
        Angle rotationToHub = Radians.of(Math.atan2(pose.getY(), pose.getX()));

        SmartDashboard.putNumber("rotation given to turret", rotationToHub.in(Radians));
        turretSub.setTurretAngle(rotationToHub);

        return true;
    }

    public void swerveAlign(Pose2d targetPose) {
        // Pose2d robotPose = swerveSub.getPose();
        Pose2d robotPose = new Pose2d();
        Transform2d vectorDifference = targetPose.minus(robotPose);
        Angle angleFieldRelative = Radians.of(Math.atan2(vectorDifference.getY(), vectorDifference.getX()));
        Angle absoluteAngle =
                angleFieldRelative.plus(Degrees.of(robotPose.getRotation().getDegrees()));
        turretSub.setTurretAngle(absoluteAngle);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
