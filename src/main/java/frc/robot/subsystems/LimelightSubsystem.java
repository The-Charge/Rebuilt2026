package frc.robot.subsystems;

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
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.LimelightConstants.StdDevConstants;
import frc.robot.constants.LimelightConstants.StdDevConstants.MegaTag1;
import frc.robot.constants.LimelightConstants.StdDevConstants.MegaTag2;
import frc.robot.utils.Logger;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.StreamMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.AprilTagFiducial;
import limelight.results.RawFiducial;

public class LimelightSubsystem extends SubsystemBase {
    private final Limelight turretLimelight;
    private final Limelight sideLimelight;

    private final LimelightPoseEstimator turretPoseEstimator;
    private final LimelightPoseEstimator sidePoseEstimator;

    private final Optional<StructArrayPublisher<Pose2d>> visionTargetsTurret;
    private final Optional<StructArrayPublisher<Pose2d>> visionTargetsSide;

    private final Optional<StructPublisher<Pose3d>> diffPublisher;

    public static record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    public LimelightSubsystem(
            String turretLimelight, String sideLimelight, Pose3d cameraOffsetTurret, Pose3d cameraOffsetSide) {
        this.turretLimelight = new Limelight("limelight-" + turretLimelight);
        this.sideLimelight = new Limelight("limelight-" + sideLimelight);

        // LimelightHelpers.setRewindEnabled(name, true); // wtf is this in YALL ??
        this.turretLimelight
                .getSettings()
                .withImuMode(LimelightSettings.ImuMode.ExternalImu)
                .withStreamMode(StreamMode.Standard)
                .withCameraOffset(cameraOffsetTurret)
                .save();
        this.sideLimelight
                .getSettings()
                .withImuMode(LimelightSettings.ImuMode.ExternalImu)
                .withStreamMode(StreamMode.Standard)
                .withCameraOffset(cameraOffsetSide)
                .save();
        this.turretPoseEstimator = this.turretLimelight.createPoseEstimator(EstimationMode.MEGATAG2);
        this.sidePoseEstimator = this.sideLimelight.createPoseEstimator(EstimationMode.MEGATAG2);

        // LimelightHelpers.SetIMUAssistAlpha(name, 0.1);
        visionTargetsTurret =
                Logger.makeStructArrayPublisher("Limelight" + turretLimelight, "visionTargetsTurret", Pose2d.struct);
        visionTargetsSide =
                Logger.makeStructArrayPublisher("Limelight" + sideLimelight, "visionTargetsSide", Pose2d.struct);

        diffPublisher = Logger.makeStructPublisher(getName(), "poseDeviation", Pose3d.struct);
    }

    private boolean throttle = false;
    private boolean isChanged = false;
    /**
     * NEED TO RUN TO BE TRUE WHEN ENABLED!!!
     * @param isEnabled
     */
    public void setThrottle(boolean isEnabled) {
        throttle = isEnabled;
        isChanged = true;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(LimelightConstants.subsystemName, this);

        // ****** FIX FIX FIX *******
        if (throttle && isChanged) {
            // turretLimelight.setThrottle(false);
            // sideLimelight.setThrottle(false);
            // turretLimelight.getSettings().
            // turretLimelight.setPipeline(1);
            // sideLimelight.setPipeline(1);
            Logger.logBool(LimelightConstants.subsystemName, "throttle", true);
            isChanged = false;
        }

        // ****** FIX FIX FIX *******
        if (!throttle && isChanged) {
            // turretLimelight.setIMUMode(1);
            // sideLimelight.setIMUMode(1);
            // turretLimelight.setThrottle(true);
            // sideLimelight.setThrottle(true);
            // turretLimelight.setPipeline(0);
            // sideLimelight.setPipeline(0);
            Logger.logBool(LimelightConstants.subsystemName, "enabled", false);
            isChanged = false;
        }

        // if (!isSeeded) {
        //     seed();
        // }

        // setRobotOrientation();

        Angle yaw = RobotContainer.getInstance()
                .swerve
                .getState()
                .Pose
                .getRotation()
                .getMeasure();

        AngularVelocity3d angularVelocity = new AngularVelocity3d(
                RobotContainer.getInstance()
                        .swerve
                        .getPigeon2()
                        .getAngularVelocityXWorld()
                        .getValue(),
                RobotContainer.getInstance()
                        .swerve
                        .getPigeon2()
                        .getAngularVelocityYWorld()
                        .getValue(),
                RobotContainer.getInstance()
                        .swerve
                        .getPigeon2()
                        .getAngularVelocityZWorld()
                        .getValue());
        // new AngularVelocity3d(new Angular, 0.0, 0.0))
        turretLimelight
                .getSettings()
                .withRobotOrientation(
                        new Orientation3d(RobotContainer.getInstance().swerve.getRotation3d(), angularVelocity))
                .save();
        sideLimelight
                .getSettings()
                .withRobotOrientation(
                        new Orientation3d(RobotContainer.getInstance().swerve.getRotation3d(), angularVelocity))
                .save();

        calcMT1diff();

        multiple();
        // logVisionTargets(); //TODO: log vision targets after migrating to YALL
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {}

    /**
     * UNUSED???? fix later ??
     */
    // public Optional<Pose3d> getTransformToTag(int id) {
    //     LimelightTarget_Fiducial[] targetFiducials = results.targets_Fiducials;

    //     for (LimelightTarget_Fiducial targetFiducial : targetFiducials) {
    //         if (targetFiducial.fiducialID == id) {
    //             Pose3d pose = targetFiducial.getTargetPose_CameraSpace();
    //             return Optional.of(pose);
    //         }
    //     }

    //     return Optional.empty();
    // }

    /**
     * Field relative pose logged
     *
     * maybe optimize this code by combining the log loops for logging turret and side??? LATER!!
     */
    public void logVisionTargetsTurret() {
        limelight.networktables.LimelightResults results =
                turretLimelight.getLatestResults().get();
        AprilTagFiducial[] targetFiducials = results.targets_Fiducials;
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

        if (visionTargetsTurret.isPresent()) {
            visionTargetsTurret.get().set(targets);
        }
    }

    /**
     * Field relative pose logged
     */
    public void logVisionTargetsSide() {
        limelight.networktables.LimelightResults results =
                sideLimelight.getLatestResults().get();
        AprilTagFiducial[] targetFiducials = results.targets_Fiducials;
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

        if (visionTargetsSide.isPresent()) {
            visionTargetsSide.get().set(targets);
        }
    }

    // public Optional<Pose3d> getMegaTag1() {
    //     Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
    //     if (pose.equals(new Pose3d())) {
    //         return Optional.empty();
    //     }
    //     return Optional.of(pose);
    // // }

    // public Optional<Pose2d> getMegaTag2() {
    //     Pose2d pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName).pose;
    //     if (pose.equals(new Pose2d())) {
    //         return Optional.empty();
    //     }
    //     return Optional.of(pose);
    // }

    // /**
    //  * YALL getVisionMeasurement
    //  *
    //  * @param swerve
    //  * @return
    //  */
    // public Optional<VisionMeasurement> getVisionMeasurementTurret(CommandSwerveDrivetrain swerve) {
    //     Optional<PoseEstimate> opt = turretPoseEstimator.getPoseEstimate();
    //     PoseEstimate poseEstimate = opt.get();
    //     if (opt.isPresent()) {
    //         if (poseEstimate.avgTagDist > LimelightConstants.kMaxDistance.in(Meters)) return Optional.empty();
    //         return getVisionMeasurementTurret(swerve, true);
    //     }
    //     return Optional.empty();
    // }

    Optional<Matrix<N3, N1>> stdDevs; // need to be camera specific when combined

    /**
     * YALL getVisionMeasurement for Turret Camera
     * NOTE:: REMOVED MAX DISTANCE!!!!
     * @param swerve
     * @param useMegaTag2
     * @return
     */
    public Optional<VisionMeasurement> getVisionMeasurementTurret(CommandSwerveDrivetrain swerve, boolean useMegaTag2) {

        Optional<PoseEstimate> visionEstimateTurret =
                turretPoseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
        visionEstimateTurret.ifPresent((PoseEstimate poseEstimate) -> {
            if (!useMegaTag2) {
                stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
            } else {
                stdDevs = calculateStdDevsMegaTag2(
                        turretPoseEstimator.getPoseEstimate().get(), swerve);
            }
        });
        if (stdDevs.isEmpty()) {
            return Optional.empty();
        } else {
            return Optional.of(new VisionMeasurement(
                    visionEstimateTurret.get().pose.toPose2d(),
                    visionEstimateTurret.get().timestampSeconds,
                    stdDevs.get()));
        }
    }

    /**
     * YALL getVisionMeasurement for Side Camera
     * @param swerve
     * @param useMegaTag2
     * @return
     */
    public Optional<VisionMeasurement> getVisionMeasurementSide(CommandSwerveDrivetrain swerve, boolean useMegaTag2) {

        Optional<PoseEstimate> visionEstimateSide =
                sidePoseEstimator.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
        visionEstimateSide.ifPresent((PoseEstimate poseEstimate) -> {
            if (!useMegaTag2) {
                stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
            } else {
                stdDevs = calculateStdDevsMegaTag2(
                        turretPoseEstimator.getPoseEstimate().get(), swerve);
            }
        });
        if (stdDevs.isEmpty()) {
            return Optional.empty();
        } else {
            return Optional.of(new VisionMeasurement(
                    visionEstimateSide.get().pose.toPose2d(),
                    visionEstimateSide.get().timestampSeconds,
                    stdDevs.get()));
        }
    }

    private void multiple() {
        CommandSwerveDrivetrain swerve = RobotContainer.getInstance().swerve;
        getVisionMeasurementTurret(swerve, true);
        getVisionMeasurementSide(swerve, true);
    }

    /**
     * NEED NULL CHECK !!!!
     * @param poseEstimate
     * @param swerve
     * @return
     */
    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag1(
            PoseEstimate poseEstimate, CommandSwerveDrivetrain swerve) {
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

    /**
     * NEED NULL CHECK !!!!
     * @param poseEstimate
     * @param swerve
     * @return
     */
    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(
            PoseEstimate poseEstimate, CommandSwerveDrivetrain swerve) {
        // if (!LimelightHelpers.validPoseEstimate(poseEstimate)) return Optional.empty();

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

    /**
     *
     * MAYBE SEEDING BACK??
     * @param imuMode
     */
    // public void seedInternalIMU(Angle yaw) {
    //     LimelightHelpers.SetIMUMode(cameraName, 1);
    //     LimelightHelpers.SetRobotOrientation(cameraName, yaw.in(Degrees), 0, 0, 0, 0, 0);
    //     LimelightHelpers.SetIMUMode(cameraName, 3);
    // }

    // public void setRobotOrientation(Angle yaw) {
    //     LimelightHelpers.SetRobotOrientation(cameraName, yaw.in(Degrees), 0, 0, 0, 0, 0);
    // }

    public void setIMUModeTurret(ImuMode imuMode) {
        turretLimelight.getSettings().withImuMode(imuMode).save();
    }

    public void setIMUModeSide(ImuMode imuMode) {
        sideLimelight.getSettings().withImuMode(imuMode).save();
    }

    // public void setThrottle(boolean enabled) {
    // LimelightHelpers.SetThrottle(cameraName, enabled ? 200 : 0);
    // }

    public void setPipelineTurret(int pipe) {
        turretLimelight.getSettings().withPipelineIndex(pipe).save();
    }

    public void setPipelineSide(int pipe) {
        sideLimelight.getSettings().withPipelineIndex(pipe).save();
    }

    // public void takeRewind() {
    //     LimelightHelpers.triggerRewindCapture(cameraName, 200.0);
    // }

    private void calcMT1diff() {
        if (diffPublisher.isEmpty()) return;

        Optional<PoseEstimate> turret = turretPoseEstimator.getPoseEstimate();
        Optional<PoseEstimate> side = sidePoseEstimator.getPoseEstimate();
        if (turret.isEmpty() || side.isEmpty()) {
            return;
        }
        Pose3d turretPose = turret.get().pose;
        Pose3d sidePose = side.get().pose;

        Pose3d diff = new Pose3d().plus(sidePose.minus(turretPose));
        Logger.logPose3dAsDoubleArray(LimelightConstants.subsystemName, "transformBetweenCameras", diff);
        diffPublisher.get().set(diff);
    }
}
