package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;
import java.util.function.Supplier;

public class AlignTurret extends Command {
    private final SwerveSubsystem swerveSub;
    private final TurretSubsystem turretSub;
    private final LimelightSubsystem limelightSub;

    private final Optional<Boolean> isRed;
    private final Optional<Supplier<Translation2d>> targetPoint;

    // Use AlignTurret to align Turret to a specified Pose (usually given by Swerve, or LimelightHelpers.getBotPose())
    public AlignTurret(
            TurretSubsystem turret,
            SwerveSubsystem swerve,
            LimelightSubsystem limelight,
            Supplier<Translation2d> point) {

        this.swerveSub = swerve;
        this.turretSub = turret;
        this.limelightSub = limelight;

        this.targetPoint = Optional.of(point);
        this.isRed = Optional.empty();

        addRequirements(turret);
    }

    // Align the turret directly to AprilTag on Hub (given by Alliance)
    public AlignTurret(
            TurretSubsystem turret, SwerveSubsystem swerve, LimelightSubsystem limelight, Alliance alliance) {

        this.swerveSub = swerve;
        this.turretSub = turret;
        this.limelightSub = limelight;

        this.isRed = Optional.of(alliance == Alliance.Red);
        this.targetPoint = Optional.empty();

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
                swerveAlign(FieldConstants.getHubLoc(isRed.get()));
            }
            return;
        }

        // If supplied given pose
        if (targetPoint.isPresent()) {
            swerveAlign(targetPoint.get().get());
        }
    }

    //
    public boolean hubTagAlign(boolean isRed) {
        // Get Detection (safe)
        // change based on which alliance
        Optional<Pose3d> poseOpt = limelightSub.getTransformToTag(FieldConstants.getHubTag(isRed));

        // Gets actual pose (safe)
        if (poseOpt.isEmpty()) return false; // TODO: log that this failed
        Pose3d pose = poseOpt.get();

        // Set turret angle to robotToHub vector
        Angle rotationToHub = Radians.of(Math.atan2(pose.getY(), pose.getX()));

        SmartDashboard.putNumber("rotation given to turret", rotationToHub.in(Radians));
        turretSub.setTurretAngle(rotationToHub);

        return true;
    }

    public void swerveAlign(Translation2d targetLoc) {
        Pose2d robotPose = swerveSub.getPose();
        Translation2d vectorDifference = targetLoc.minus(robotPose.getTranslation());
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
