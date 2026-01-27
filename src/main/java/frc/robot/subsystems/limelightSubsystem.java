package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.LimelightConstants.StdDevConstants;
import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
    private final String name;
    // CommandSwerveDrivetrain drivetrain;

    public LimelightSubsystem(String name) {
        this.name = "limelight-" + name;
    }

    public Optional<Pose2d> getRawPosition() {
        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(name);
        if (pose == null) return Optional.empty();
        return Optional.of(pose);
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

    public Optional<Matrix<N3, N1>> calculateStdDevsMegaTag1(
            LimelightHelpers.PoseEstimate poseEstimate, SwerveSubsystem swerve) {
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
        double transStdDev = StdDevConstants.MegaTag1.kInitialValue;

        if (poseEstimate.tagCount == 1
                && poseEstimate.rawFiducials.length == 1) { // single tag TODO: why are two checks needed?
            RawFiducial singleTag = poseEstimate.rawFiducials[0];
            if (LimelightConstants.kVisionDiagnostics)
                SmartDashboard.putNumber(
                        "VisionDiagnostics/" + name + "/single tag pose ambiguity", singleTag.ambiguity);
            if (singleTag.ambiguity > 0.7 || singleTag.distToCamera > 5) {
                return Optional.empty(); // don't trust if too ambiguous or too far
            }
            transStdDev +=
                    StdDevConstants.MegaTag1.kSingleTagPunishment; // megatag1 performs much worse with only one tag
        }
        transStdDev -= Math.min(poseEstimate.tagCount, 4) * StdDevConstants.MegaTag1.kTagCountReward;
        transStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag1.kAverageDistancePunishment;
        transStdDev += swerve.getSpeed() * StdDevConstants.MegaTag1.kRobotSpeedPunishment;

        transStdDev = Math.max(transStdDev, 0.05); // make sure we aren't putting all our trust in vision

        double rotStdDev = LimelightConstants.krotStdDev; // we want to get the rotation from megatag1

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    }

    public Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(
            LimelightHelpers.PoseEstimate poseEstimate, SwerveSubsystem swerve) {
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
        if (swerve.getAngularVelocity().abs(Units.DegreesPerSecond) > LimelightConstants.kMaxAngularSpeed)
            return Optional.empty(); // don't trust if turning too fast
        if (poseEstimate.avgTagDist > 8) return Optional.empty();

        double transStdDev = StdDevConstants.MegaTag2.kInitialValue;

        if (poseEstimate.tagCount > 1)
            transStdDev -= StdDevConstants.MegaTag2.kMultipleTagsBonus; // TODO: is this even needed?
        transStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag2.kAverageDistancePunishment;
        transStdDev += swerve.getSpeed() * StdDevConstants.MegaTag2.kRobotSpeedPunishment;

        transStdDev = Math.max(transStdDev, 0.05); // make sure we aren't putting all our trust in vision

        double rotStdDev = Double.MAX_VALUE; // never trust rotation under any circumstances

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    }
    public Optional<PoseEstimate> getVisionOnlyPos() {
        return Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue(this.name));
    }
    // public Optional<PoseEstimate> getVisionMeasurement(SwerveSubsystem swerve, boolean useMegaTag2) {
    //     LimelightHelpers.SetRobotOrientation(name, 0, 0, 0, 0, 0, 0);
    // }
}
