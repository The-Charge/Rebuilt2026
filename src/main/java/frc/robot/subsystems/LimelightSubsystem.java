package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    private final String ll_name;

    public static record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    public LimelightSubsystem(String name, Pose3d cameraOffset) {
        this.ll_name = "limelight-" + name;
        setCameraOffset(cameraOffset);
    }
    public void setCameraOffset(Pose3d cameraOffset) {
        LimelightHelpers.setCameraPose_RobotSpace(
                ll_name,
                cameraOffset.getX(),
                cameraOffset.getY(),
                cameraOffset.getZ(),
                Degrees.convertFrom(cameraOffset.getRotation().getX(), Radians),
                Degrees.convertFrom(cameraOffset.getRotation().getY(), Radians),
                Degrees.convertFrom(cameraOffset.getRotation().getZ(), Radians));
    }

    public Optional<VisionMeasurement> getVisionMeasurement(SwerveSubsystem swerve) {
        boolean useMegaTag2 = true;
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll_name);
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
        if (poseEstimate.avgTagDist > LimelightConstants.kMaxDistance) return Optional.empty();
        final boolean twoOrMoreTags = poseEstimate.tagCount >= 2;
        final boolean closeEnough = poseEstimate.avgTagDist < LimelightConstants.kMaxDistanceForMegaTag1;
        final double robotSpeed = swerve.getSpeed(); // get swerve speed from ctre
        final boolean movingSlowEnough = robotSpeed < LimelightConstants.kMaxSpeedForMegaTag1;
        final boolean CAN_GET_GOOD_HEADING = twoOrMoreTags && movingSlowEnough && closeEnough;
        if (!CAN_GET_GOOD_HEADING) return Optional.empty();
        if (CAN_GET_GOOD_HEADING) useMegaTag2 = false;
        return getVisionMeasurement(swerve, useMegaTag2);
    }

    public Optional<VisionMeasurement> getVisionMeasurement(SwerveSubsystem swerve, boolean useMegaTag2) {
        PoseEstimate poseEstimate;
        Optional<Matrix<N3, N1>> stdDevs;
        if (!useMegaTag2) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll_name);
            stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
        } else {
            LimelightHelpers.SetRobotOrientation(ll_name, swerve.getHeading().getDegrees(), 0, 0, 0, 0, 0);
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll_name);
            stdDevs = calculateStdDevsMegaTag2(poseEstimate, swerve);
        }
        if (stdDevs.isEmpty()) {
            return Optional.empty();
        } else {
            return Optional.of(new VisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, stdDevs.get()));
        }
    }

    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag1(
            LimelightHelpers.PoseEstimate poseEstimate, SwerveSubsystem swerve) {
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();
        double transStdDev = StdDevConstants.MegaTag1.kInitialValue;

        if (poseEstimate.tagCount == 1
                && poseEstimate.rawFiducials.length == 1) { // single tag TODO: why are two checks needed?
            RawFiducial singleTag = poseEstimate.rawFiducials[0];
            if (LimelightConstants.kVisionDiagnostics)
                SmartDashboard.putNumber(
                        "VisionDiagnostics/" + ll_name + "/single tag pose ambiguity", singleTag.ambiguity);
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

    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(
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


}
