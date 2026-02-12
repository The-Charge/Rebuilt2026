package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.LimelightConstants.StdDevConstants;
import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
    private final String cameraName;

    public static record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    public LimelightSubsystem(String name, Pose3d cameraOffset) {
        this.cameraName = "limelight-" + name;
        setCameraOffset(cameraOffset);
    }

    public void setCameraOffset(Pose3d cameraOffset) {
        LimelightHelpers.setCameraPose_RobotSpace(
                cameraName,
                cameraOffset.getX(),
                cameraOffset.getY(),
                cameraOffset.getZ(),
                Degrees.convertFrom(cameraOffset.getRotation().getX(), Radians),
                Degrees.convertFrom(cameraOffset.getRotation().getY(), Radians),
                Degrees.convertFrom(cameraOffset.getRotation().getZ(), Radians));
    }

    public Optional<Pose3d> getTransformToTag(int id) {
        // Pose3d tarpose = LimelightHelpers.getTargetPose3d_CameraSpace(ll_name);
        // SmartDashboard.putNumber("Y pose of target", tarpose.getY());

        LimelightResults results = LimelightHelpers.getLatestResults(cameraName);
        LimelightTarget_Fiducial[] targetFiducials = results.targets_Fiducials;
        // SmartDashboard.putString("somethingqq", "" + results.getBotPose3d_wpiBlue().getX());
        // SmartDashboard.putString("Number of tags", fiducialss.length + "");

        for (LimelightTarget_Fiducial targetFiducial : targetFiducials) {
            // SmartDashboard.putNumber(
            //         i.fiducialID + "# tag", i.getTargetPose_CameraSpace().getY());
            if (targetFiducial.fiducialID == id) {
                Pose3d pose = targetFiducial.getTargetPose_CameraSpace();
                SmartDashboard.putNumber("ret", Math.atan2(pose.getY(), pose.getX()));
                return Optional.of(pose);
            }
        }

        return Optional.empty();
    }

    public Optional<VisionMeasurement> getVisionMeasurement(SwerveSubsystem swerve) {
        boolean useMegaTag2 = true;
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();

        SmartDashboard.putNumber("limesub", poseEstimate.tagCount);
        if (poseEstimate.avgTagDist > LimelightConstants.kMaxDistance.in(Meters)) return Optional.empty();

        final boolean twoOrMoreTags = poseEstimate.tagCount >= 2;
        final boolean closeEnough = poseEstimate.avgTagDist < LimelightConstants.kMaxDistanceForMegaTag1.in(Meters);
        final double robotSpeed = swerve.getRobotVelocity().vxMetersPerSecond; // get swerve speed from ctre
        final boolean movingSlowEnough = robotSpeed < LimelightConstants.kMaxSpeedForMegaTag1.in(MetersPerSecond);

        SmartDashboard.putBoolean("twoOrMoreTags", twoOrMoreTags);
        SmartDashboard.putBoolean("closeEnough", closeEnough);
        SmartDashboard.putBoolean("robotSpeed", movingSlowEnough);

        final boolean CAN_GET_GOOD_HEADING = twoOrMoreTags && movingSlowEnough && closeEnough;
        // final boolean CAN_GET_GOOD_HEADING = closeEnough;
        // if (!CAN_GET_GOOD_HEADING) return Optional.empty();
        if (CAN_GET_GOOD_HEADING) useMegaTag2 = false;

        return getVisionMeasurement(swerve, useMegaTag2);
    }

    public Optional<VisionMeasurement> getVisionMeasurement(SwerveSubsystem swerve, boolean useMegaTag2) {
        PoseEstimate poseEstimate;
        Optional<Matrix<N3, N1>> stdDevs;

        if (!useMegaTag2) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
            stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
        } else {
            LimelightHelpers.SetRobotOrientation(cameraName, swerve.getHeading().getDegrees(), 0, 0, 0, 0, 0);
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
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

        double translationalStdDev = StdDevConstants.MegaTag1.kInitialValue;

        if (poseEstimate.tagCount == 1
                && poseEstimate.rawFiducials.length == 1) { // single tag TODO: why are two checks needed?
            RawFiducial singleTag = poseEstimate.rawFiducials[0];

            if (LimelightConstants.isLoggingVisionDiagnostics) {
                SmartDashboard.putNumber(
                        "VisionDiagnostics/" + cameraName + "/single tag pose ambiguity", singleTag.ambiguity);
            }

            if (singleTag.ambiguity > 0.7 || singleTag.distToCamera > 5) {
                return Optional.empty(); // don't trust if too ambiguous or too far
            }

            // megatag1 performs much worse with only one tag
            translationalStdDev += StdDevConstants.MegaTag1.kSingleTagPunishment;
        }
        translationalStdDev -= Math.min(poseEstimate.tagCount, 4) * StdDevConstants.MegaTag1.kTagCountReward;
        translationalStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag1.kAverageDistancePunishment;
        translationalStdDev +=
                swerve.getRobotVelocity().vxMetersPerSecond * StdDevConstants.MegaTag1.kRobotSpeedPunishment;

        // make sure we aren't putting all our trust in vision
        translationalStdDev = Math.max(translationalStdDev, 0.05);

        double rotStdDev = LimelightConstants.krotStdDev; // we want to get the rotation from megatag1

        return Optional.of(VecBuilder.fill(translationalStdDev, translationalStdDev, rotStdDev));
        // return Optional.of(VecBuilder.fill(0, 0, 0));

    }

    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(
            LimelightHelpers.PoseEstimate poseEstimate, SwerveSubsystem swerve) {
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();

        boolean isGoingTooFast = Math.abs(swerve.getRobotVelocity().omegaRadiansPerSecond)
                > LimelightConstants.kMaxAngularSpeed.in(RadiansPerSecond);
        if (isGoingTooFast) return Optional.empty();

        if (poseEstimate.avgTagDist > 8) return Optional.empty();

        double transStdDev = StdDevConstants.MegaTag2.kInitialValue;

        if (poseEstimate.tagCount > 1) transStdDev -= StdDevConstants.MegaTag2.kMultipleTagsBonus;
        transStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag2.kAverageDistancePunishment;
        transStdDev += swerve.getRobotVelocity().vxMetersPerSecond * StdDevConstants.MegaTag2.kRobotSpeedPunishment;

        transStdDev = Math.max(transStdDev, 0.05); // make sure we aren't putting all our trust in vision

        double rotStdDev = Double.MAX_VALUE; // never trust rotation under any circumstances

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    }
}
