package frc.robot.utils;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.LimelightConstants.StdDevConstants;
import frc.robot.constants.LimelightConstants.StdDevConstants.MegaTag1;
import frc.robot.constants.LimelightConstants.StdDevConstants.MegaTag2;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

public class VisionUtils {
    public static record VisionMeasurement(Pose3d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    /**
     * Autodetects Megatag Mode
     * @return
     */
    public static Optional<Matrix<N3, N1>> calcStdDevs(PoseEstimate poseEstimate, ChassisSpeeds speeds) {
        if (LimelightConstants.activeMode.equals(LimelightConstants.Mode.MT2)) {
            return calcStdDevsMT2(poseEstimate, speeds);
        } else {
            return calcStdDevsMT1(poseEstimate, speeds);
        }
    }
    /**
     * YALL MT1 std
     * @param poseEstimate
     * @param swerve
     * @return
     */
    public static Optional<Matrix<N3, N1>> calcStdDevsMT1(
            PoseEstimate poseEstimate, ChassisSpeeds speeds) {
        if (!validPoseEstimate(poseEstimate)) return Optional.empty();

        // Optional<PoseEstimate> opt = turretPoseEstimator.getPoseEstimate();
        // PoseEstimate poseEstimate = opt.get();
        // if (opt.isEmpty()) return Optional.empty();

        double translationalStdDev = StdDevConstants.MegaTag1.kInitialValue;

        if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) { // single tag
            RawFiducial singleTag = poseEstimate.rawFiducials[0];

            if (singleTag.ambiguity > 0.7 || singleTag.distToCamera > 5) {
                return Optional.empty(); // don't trust if too ambiguous or too far
            }

            // megatag1 performs much worse with only one tag
            translationalStdDev += StdDevConstants.MegaTag1.kSingleTagPunishment;
        }

        translationalStdDev -= Math.min(poseEstimate.tagCount, 4) * StdDevConstants.MegaTag1.kTagCountReward;
        translationalStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag1.kAverageDistancePunishment;
        translationalStdDev += Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                * StdDevConstants.MegaTag1.kRobotSpeedPunishment;

        // make sure we aren't putting all our trust in vision
        translationalStdDev = Math.max(translationalStdDev, MegaTag1.kMinStd);

        double rotStdDev = LimelightConstants.kRotStdDev; // we want to get the rotation from megatag1

        return Optional.of(VecBuilder.fill(translationalStdDev, translationalStdDev, rotStdDev));
    }

    /**
     * YALL std mt2
     * @param poseEstimate
     * @param swerve
     * @return
     */
    public static Optional<Matrix<N3, N1>> calcStdDevsMT2(
            PoseEstimate poseEstimate, ChassisSpeeds speeds) {
        if (!validPoseEstimate(poseEstimate)) return Optional.empty();
        boolean isGoingTooFast = Math.abs(speeds.omegaRadiansPerSecond)
                > LimelightConstants.kMaxAngularSpeed.in(RadiansPerSecond);
        if (isGoingTooFast) return Optional.empty();

        if (poseEstimate.avgTagDist > 8) return Optional.empty();

        double transStdDev = StdDevConstants.MegaTag2.kInitialValue;

        if (poseEstimate.tagCount > 1) transStdDev -= StdDevConstants.MegaTag2.kMultipleTagsBonus;
        transStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag2.kAverageDistancePunishment;
        transStdDev += Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                * StdDevConstants.MegaTag2.kRobotSpeedPunishment;

        transStdDev = Math.max(transStdDev, MegaTag2.kMinStd); // make sure we aren't putting all our trust in vision

        double rotStdDev = LimelightConstants.kRotStdDev / 4; // never trust rotation under any circumstances, but
        // maybe do
        // double rotStdDev = Double.MAX_VALUE;

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    }
    
    /**
     * Returns whether given pose estimate is valid. First check if optional is empty!
     * @param pe
     * @return
     */
    public static boolean validPoseEstimate(PoseEstimate pe) {
        if (pe == null || !pe.hasData || pe.rawFiducials == null || pe.rawFiducials.length == 0) return false;
        return true;
    }

    /**
     * Checks if the pose estimate is is flying, or outside of the field
     * @param pe
     * @return true if valid, false if rejected
     */
    public static boolean isFiltered(VisionMeasurement vm) {
        if (Math.abs(vm.pose.getZ()) > LimelightConstants.StdDevConstants.Filter.kMaxHeight
                || vm.pose.getX() < 0
                || vm.pose.getMeasureX().gt(FieldConstants.fieldXBound)
                || vm.pose.getY() < 0
                || vm.pose.getMeasureY().gt(FieldConstants.fieldYBound)) return true;
        else {
            return false;
        }
    }
}
