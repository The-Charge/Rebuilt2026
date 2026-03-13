package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.LimelightConstants.StdDevConstants;
import frc.robot.constants.LimelightConstants.StdDevConstants.MegaTag1;
import frc.robot.constants.LimelightConstants.StdDevConstants.MegaTag2;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightResults;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import frc.robot.utils.Logger;
import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
    private final String cameraName;
    private final Optional<StructArrayPublisher<Pose2d>> visionTargets;

    public static record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    public LimelightSubsystem(String name, Pose3d cameraOffset) {
        this.cameraName = "limelight-" + name;
        setCameraOffset(cameraOffset);
        LimelightHelpers.setRewindEnabled(name, true);
        LimelightHelpers.SetIMUMode(name, 3);
        // LimelightHelpers.SetIMUAssistAlpha(name, 0.1);
        visionTargets = Logger.makeStructArrayPublisher("Limelight" + name, "visionTargets", Pose2d.struct);
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(cameraName, this);
        // logVisionTargets(); //TODO: log vision targets after migrating to YALL
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {}

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
        LimelightResults results = LimelightHelpers.getLatestResults(cameraName);
        LimelightTarget_Fiducial[] targetFiducials = results.targets_Fiducials;

        for (LimelightTarget_Fiducial targetFiducial : targetFiducials) {
            if (targetFiducial.fiducialID == id) {
                Pose3d pose = targetFiducial.getTargetPose_CameraSpace();
                return Optional.of(pose);
            }
        }

        return Optional.empty();
    }

    public void logVisionTargets() {
        LimelightResults results = LimelightHelpers.getLatestResults(cameraName);
        LimelightTarget_Fiducial[] targetFiducials = results.targets_Fiducials;
        Pose2d[] targets = new Pose2d[targetFiducials.length];
        // Logger.println(Integer.toString(targetFiducials.length));

        for (int i = 0; i < targets.length; i++) {
            Pose2d targetFieldSpace = targetFiducials[i]
                    .getTargetPose_RobotSpace2D()
                    .plus(new Transform2d(
                            RobotContainer.getInstance().swerve.getState().Pose.getTranslation(),
                            RobotContainer.getInstance().swerve.getState().Pose.getRotation()));
            targets[i] = targetFieldSpace;
        }

        if (visionTargets.isPresent()) {
            visionTargets.get().set(targets);
        }
    }

    public Optional<Pose3d> getMegaTag1() {
        Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
        if (pose.equals(new Pose3d())) {
            return Optional.empty();
        }
        return Optional.of(pose);
    }

    public Optional<Pose2d> getMegaTag2() {
        Pose2d pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName).pose;
        if (pose.equals(new Pose2d())) {
            return Optional.empty();
        }
        return Optional.of(pose);
    }

    public Optional<VisionMeasurement> getVisionMeasurement(CommandSwerveDrivetrain swerve) {
        final PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();

        if (poseEstimate.avgTagDist > LimelightConstants.kMaxDistance.in(Meters)) return Optional.empty();

        return getVisionMeasurement(swerve, true);
    }

    public Optional<VisionMeasurement> getVisionMeasurement(CommandSwerveDrivetrain swerve, boolean useMegaTag2) {
        PoseEstimate poseEstimate;
        Optional<Matrix<N3, N1>> stdDevs;

        if (!useMegaTag2) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
            stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
        } else {
            // LimelightHelpers.SetRobotOrientation(cameraName, swerve.getStateCopy().RawHeading.getDegrees(), 0, 0, 0,
            // 0, 0);
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
            LimelightHelpers.PoseEstimate poseEstimate, CommandSwerveDrivetrain swerve) {
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();

        double translationalStdDev = StdDevConstants.MegaTag1.kInitialValue;

        if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) { // single tag
            RawFiducial singleTag = poseEstimate.rawFiducials[0];

            if (singleTag.ambiguity > 0.7 || singleTag.distToCamera > 5) {
                return Optional.empty(); // don't trust if too ambiguous or too far
            }

            // megatag1 performs much worse with only one tag
            translationalStdDev += StdDevConstants.MegaTag1.kSingleTagPunishment;
        }

        ChassisSpeeds speed = swerve.getStateCopy().Speeds;

        translationalStdDev -= Math.min(poseEstimate.tagCount, 4) * StdDevConstants.MegaTag1.kTagCountReward;
        translationalStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag1.kAverageDistancePunishment;
        translationalStdDev += Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond)
                * StdDevConstants.MegaTag1.kRobotSpeedPunishment;

        // make sure we aren't putting all our trust in vision
        translationalStdDev = Math.max(translationalStdDev, MegaTag1.kMinStd);

        double rotStdDev = LimelightConstants.krotStdDev; // we want to get the rotation from megatag1

        return Optional.of(VecBuilder.fill(translationalStdDev, translationalStdDev, rotStdDev));
    }

    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(
            LimelightHelpers.PoseEstimate poseEstimate, CommandSwerveDrivetrain swerve) {
        if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();

        boolean isGoingTooFast = Math.abs(swerve.getStateCopy().Speeds.omegaRadiansPerSecond)
                > LimelightConstants.kMaxAngularSpeed.in(RadiansPerSecond);
        if (isGoingTooFast) return Optional.empty();

        if (poseEstimate.avgTagDist > 8) return Optional.empty();

        double transStdDev = StdDevConstants.MegaTag2.kInitialValue;

        ChassisSpeeds speed = swerve.getStateCopy().Speeds;

        if (poseEstimate.tagCount > 1) transStdDev -= StdDevConstants.MegaTag2.kMultipleTagsBonus;
        transStdDev += poseEstimate.avgTagDist * StdDevConstants.MegaTag2.kAverageDistancePunishment;
        transStdDev += Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond)
                * StdDevConstants.MegaTag2.kRobotSpeedPunishment;

        transStdDev = Math.max(transStdDev, MegaTag2.kMinStd); // make sure we aren't putting all our trust in vision

        // double rotStdDev = LimelightConstants.krotStdDev / 4; // never trust rotation under any circumstances, but
        // maybe do
        double rotStdDev = Double.MAX_VALUE;

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    }

    public void seedInternalIMU(Angle yaw) {
        LimelightHelpers.SetIMUMode(cameraName, 1);
        LimelightHelpers.SetRobotOrientation(cameraName, yaw.in(Degrees), 0, 0, 0, 0, 0);
        LimelightHelpers.SetIMUMode(cameraName, 3);
    }

    public void setRobotOrientation(Angle yaw) {
        LimelightHelpers.SetRobotOrientation(cameraName, yaw.in(Degrees), 0, 0, 0, 0, 0);
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(cameraName, mode);
    }

    public void setThrottle(boolean enabled) {
        LimelightHelpers.SetThrottle(cameraName, enabled ? 200 : 0);
    }

    public void setPipeline(int pipe) {
        LimelightHelpers.setPipelineIndex(cameraName, pipe);
    }

    public void takeRewind() {
        LimelightHelpers.triggerRewindCapture(cameraName, 155.0); // record starting 5 seconds before match starts
    }
}
