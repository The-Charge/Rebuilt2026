package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.units.TurretAngle;
import frc.robot.utils.Logger;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AlignTurret extends Command {
    private final CommandSwerveDrivetrain swerveSub;
    private final TurretSubsystem turretSub;
    private final LimelightSubsystem limelightSub;

    private final Optional<BooleanSupplier> isRed;
    private final Optional<Supplier<Translation2d>> targetPoint;

    private AlignTurret(
            TurretSubsystem turret,
            CommandSwerveDrivetrain swerve,
            LimelightSubsystem limelight,
            Optional<Supplier<Translation2d>> point,
            Optional<Supplier<Alliance>> alliance) {

        this.swerveSub = swerve;
        this.turretSub = turret;
        this.limelightSub = limelight;

        if (point.isEmpty()) {
            targetPoint = Optional.empty();
        } else {
            targetPoint = Optional.of(point.get());
        }
        if (alliance.isEmpty()) {
            isRed = Optional.empty();
        } else {
            isRed = Optional.of(() -> alliance.get().get() == Alliance.Red);
        }

        addRequirements(turret);
    }

    public static AlignTurret atPoint(
            TurretSubsystem turret,
            CommandSwerveDrivetrain swerve,
            LimelightSubsystem limelight,
            Supplier<Translation2d> point) {
        return new AlignTurret(turret, swerve, limelight, Optional.of(point), Optional.empty());
    }

    public static AlignTurret atHub(
            TurretSubsystem turret,
            CommandSwerveDrivetrain swerve,
            LimelightSubsystem limelight,
            Supplier<Alliance> alliance) {
        return new AlignTurret(turret, swerve, limelight, Optional.empty(), Optional.of(alliance));
    }

    @Override
    public void initialize() {
        // SmartDashboard.putNumber("turretAngleDeg", 60);
    }

    @Override
    public void execute() {
        if (isRed.isPresent()) {
            // targeting alliance hub

            boolean succeeded = hubTagAlign(isRed.get().getAsBoolean());
            if (succeeded) {
                Logger.logString(getName(), "alignMethod", "hubTag");
            } else {
                // If can't see hub tag, use swervePose
                swerveAlign(FieldConstants.getHubLoc(isRed.get().getAsBoolean()));
                Logger.logString(getName(), "alignMethod", "swerve");
            }
            return;
        }

        if (targetPoint.isPresent()) {
            // targeting given point
            swerveAlign(targetPoint.get().get());
            Logger.logString(getName(), "alignMethod", "swerve");
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
        Angle angleToHub = Radians.of(Math.atan2(pose.getY(), pose.getX()));

        turretSub.setTurretAngle(TurretAngle.fromMechanismAngle(angleToHub));

        return true;
    }

    public void swerveAlign(Translation2d targetLoc) {
        Pose2d robotPose = swerveSub.getStateCopy().Pose;
        Translation2d vectorDifference = targetLoc.minus(robotPose.getTranslation());
        Angle fieldCentricAngle = Radians.of(Math.atan2(vectorDifference.getY(), vectorDifference.getX()));
        Angle robotCentricAngle = fieldCentricAngle.plus(robotPose.getRotation().getMeasure());

        turretSub.setTurretAngle(TurretAngle.fromMechanismAngle(robotCentricAngle));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
