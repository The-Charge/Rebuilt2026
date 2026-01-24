package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.LimelightConstants;
import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
    private final String name;
    // CommandSwerveDrivetrain drivetrain;

    public LimelightSubsystem(String name) {
        this.name = "limelight-" + name;
    }

    public Optional<PoseEstimate> getVisionMeasurement() {
        boolean useMegaTag2 = true;
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
        if (poseEstimate.avgTagDist > LimelightConstants.kMaxDistance) return Optional.empty();
        final boolean twoOrMoreTags = poseEstimate.tagCount >= 2;
        final boolean closeEnough = poseEstimate.avgTagDist < LimelightConstants.kMaxDistanceForMegaTag1;
        final double robotSpeed = 0; // get swerve speed from ctre
        final boolean movingSlowEnough = robotSpeed < LimelightConstants.kMaxSpeedForMegaTag1;
        final boolean CAN_GET_GOOD_HEADING = twoOrMoreTags && movingSlowEnough && closeEnough;
        if (!CAN_GET_GOOD_HEADING) return Optional.empty();
        if (CAN_GET_GOOD_HEADING) useMegaTag2 = false;
        return Optional.empty();
    }

    // public Optional<PoseEstimate> getVisionMeasurement(SwerveSubsystem swerve, boolean useMegaTag2) {
    //     LimelightHelpers.SetRobotOrientation(name, 0, 0, 0, 0, 0, 0);
    // }
}
