package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.HoodPos;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;

public class ShootTurret extends Command {

    public ShooterSubsystem.HoodPos hoodPos;

    private final ShooterSubsystem shooterSub;
    private final LimelightSubsystem vSub;
    private final SwerveSubsystem swerveSub;
    private final boolean isRed;

    public ShootTurret(ShooterSubsystem shootSub, LimelightSubsystem vSub, SwerveSubsystem swerveSub, boolean isRed) {
        this.shooterSub = shootSub;
        this.vSub = vSub;
        this.swerveSub = swerveSub;
        this.isRed = isRed;
        addRequirements(shooterSub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Pose2d poseToHub;

        // Use HubTag directly for distance
        Optional<Pose3d> hubTagPose = vSub.getTransformToTag(FieldConstants.getHubTag(isRed));

        // Otherwise use swerve positioning
        if (!hubTagPose.isEmpty()) {
            poseToHub = hubTagPose.get().toPose2d();
        } else {
            // Get Swerve Distance to Hub

            // Hub Pose - Robot Swerve Pose
            Transform2d poseToHubTransform = FieldConstants.getHubPos(isRed).minus(swerveSub.getPose());
            // Transform2d -> Pose2d
            poseToHub = new Pose2d(poseToHubTransform.getX(), poseToHubTransform.getY(), new Rotation2d());
            return;
        }
        // Magnitude of Robot -> Hub vector
        double distance = poseToHub.getTranslation().getNorm();

        if (distance > ShooterConstants.hoodPosThreshold) {
            // if far, shoot low and far
            shooterSub.setHoodPos(HoodPos.DOWN);
        } else {
            // if close, shoot high
            shooterSub.setHoodPos(HoodPos.UP);
        }

        shooterSub.shoot(RPM.of(ShooterConstants.distanceToRPMPlot.get(distance))); // Used to be: RPM.of(10)
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
