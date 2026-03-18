package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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

    private final LimelightPoseEstimator turretPoseEstimatorMT2;
    private final LimelightPoseEstimator sidePoseEstimatorMT2;

    private final LimelightPoseEstimator turretPoseEstimatorMT1;
    private final LimelightPoseEstimator sidePoseEstimatorMT1;

    private final Optional<StructArrayPublisher<Pose2d>> visionTargetsTurret;
    private final Optional<StructArrayPublisher<Pose2d>> visionTargetsSide;

    private final Optional<StructPublisher<Pose3d>> diffPublisher;

    public static record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    private boolean isSeeded = false;

    public LimelightSubsystem(
            String turretLimelight, String sideLimelight, Pose3d cameraOffsetTurret, Pose3d cameraOffsetSide) {
        this.turretLimelight = new Limelight("limelight-" + turretLimelight);
        this.sideLimelight = new Limelight("limelight-" + sideLimelight);

        initLimelights(cameraOffsetTurret, cameraOffsetSide);

        this.turretPoseEstimatorMT2 = this.turretLimelight.createPoseEstimator(EstimationMode.MEGATAG2);
        this.sidePoseEstimatorMT2 = this.sideLimelight.createPoseEstimator(EstimationMode.MEGATAG2);

        this.turretPoseEstimatorMT1 = this.turretLimelight.createPoseEstimator(EstimationMode.MEGATAG1);
        this.sidePoseEstimatorMT1 = this.sideLimelight.createPoseEstimator(EstimationMode.MEGATAG1);
        
        visionTargetsTurret =
                Logger.makeStructArrayPublisher("Limelight" + turretLimelight, "visionTargetsTurret", Pose2d.struct);
        visionTargetsSide =
                Logger.makeStructArrayPublisher("Limelight" + sideLimelight, "visionTargetsSide", Pose2d.struct);

        diffPublisher = Logger.makeStructPublisher(getName(), "poseDeviation", Pose3d.struct);
    }

    private void initLimelights(Pose3d cameraOffsetTurret, Pose3d cameraOffsetSide) {
        // LimelightHelpers.setRewindEnabled(name, true); // wtf is this in YALL ??
        this.turretLimelight
                .getSettings()
                .withImuMode(LimelightSettings.ImuMode.ExternalImu)
                .withStreamMode(StreamMode.Standard)
                .withCameraOffset(cameraOffsetTurret)
                .withImuAssistAlpha(LimelightConstants.imuAssistAlpha)
                .save();
        this.sideLimelight
                .getSettings()
                .withImuMode(LimelightSettings.ImuMode.ExternalImu)
                .withStreamMode(StreamMode.Standard)
                .withCameraOffset(cameraOffsetSide)
                .withImuAssistAlpha(LimelightConstants.imuAssistAlpha)
                .save();
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

        setRobotOrientationSwerve();
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

    
    private void setRobotOrientationSwerve() {

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
    }
    private void setRobotOrientationAbsolute(Angle yaw) {

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
        turretLimelight
                .getSettings()
                .withRobotOrientation(
                        new Orientation3d(new Rotation3d(0, 0, yaw.in(Radians)), angularVelocity))
                .save();
        sideLimelight
                .getSettings()
                .withRobotOrientation(
                        new Orientation3d(new Rotation3d(0, 0, yaw.in(Radians)), angularVelocity))
                .save();
    }

    /**
     * Seeds from MT1, then sets mode to imuMode
     */
    public void seedMT1(ImuMode imuMode) {
        CommandSwerveDrivetrain swerve = RobotContainer.getInstance().swerve;

        Optional<VisionMeasurement> MT1turret = getVisionMeasurementTurret(swerve, false);
        Optional<VisionMeasurement> MT1side = getVisionMeasurementSide(swerve, false);
        if (MT1turret.isPresent() || MT1side.isPresent()) {
            Angle rots;
            if (MT1turret.isPresent() && MT1side.isPresent()) {
                if (MT1turret.get().stdDevs().get(0, 0)
                        > MT1side.get().stdDevs().get(0, 0)) {
                    rots = MT1turret.get().pose().getRotation().getMeasure();

                } else {
                    rots = MT1side.get().pose().getRotation().getMeasure();
                }
            } else if (MT1turret.isPresent()) {
                rots = MT1turret.get().pose().getRotation().getMeasure();
            } else {
                rots = MT1side.get().pose().getRotation().getMeasure();
            }
            seedBothInternalIMU(rots, imuMode);
            isSeeded = true;
        }
    }
    /**
     *
     * @param imuMode
     */
    public void seedBothInternalIMU(Angle yaw, ImuMode imuMode) {
        turretLimelight.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
        sideLimelight.getSettings().withImuMode(ImuMode.SyncInternalImu).save();

        setRobotOrientationAbsolute(yaw);

        sideLimelight.getSettings().withImuMode(imuMode).save();
        turretLimelight.getSettings().withImuMode(imuMode).save();
    }

    public boolean getIsSeeded() {
        return isSeeded;
    }
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

    public Optional<PoseEstimate> getMegaTag1Turret() {
        Optional<PoseEstimate> poseEstimate = turretPoseEstimatorMT1.getPoseEstimate();
        if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
        return poseEstimate;
    }
    public Optional<PoseEstimate> getMegTag1Side() {
        Optional<PoseEstimate> poseEstimate = sidePoseEstimatorMT1.getPoseEstimate();
        if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
        return poseEstimate;
    }
    public Optional<PoseEstimate> getMegaTag2Turret() {
        Optional<PoseEstimate> poseEstimate = turretPoseEstimatorMT2.getPoseEstimate();
        if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
        return poseEstimate;
    }
    public Optional<PoseEstimate> getMegaTag2Side() {
        Optional<PoseEstimate> poseEstimate = sidePoseEstimatorMT2.getPoseEstimate();
        if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
        return poseEstimate;
    }
    Optional<Matrix<N3, N1>> stdDevs; // need to be camera specific when combined

    /**
     * YALL getVisionMeasurement for Turret Camera
     * NOTE:: REMOVED MAX DISTANCE!!!!
     * NEED TO MAKE BOOL MEGATAG2 DO SOMETHING!!
     * @param swerve
     * @param useMegaTag2
     * @return
     */
    public Optional<VisionMeasurement> getVisionMeasurementTurret(CommandSwerveDrivetrain swerve, boolean useMegaTag2) {

        Optional<PoseEstimate> visionEstimateTurret =
                turretPoseEstimatorMT2.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
        visionEstimateTurret.ifPresent((PoseEstimate poseEstimate) -> {
            if (!useMegaTag2) {
                stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
            } else {
                stdDevs = calculateStdDevsMegaTag2(
                        turretPoseEstimatorMT2.getPoseEstimate().get(), swerve);
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
                sidePoseEstimatorMT2.getPoseEstimate(); // BotPose.BLUE_MEGATAG2.get(limelight);
        visionEstimateSide.ifPresent((PoseEstimate poseEstimate) -> {
            if (!useMegaTag2) {
                stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
            } else {
                stdDevs = calculateStdDevsMegaTag2(
                        turretPoseEstimatorMT2.getPoseEstimate().get(), swerve);
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
        Optional<VisionMeasurement> vmt = getVisionMeasurementTurret(swerve, true);
        Optional<VisionMeasurement> vms = getVisionMeasurementSide(swerve, true);
        if (!vmt.isEmpty()) {
            swerve.addVisionMeasurement(vmt.get().pose, vmt.get().timestamp, vmt.get().stdDevs);
        }
        if (!vms.isEmpty()) {
            swerve.addVisionMeasurement(vms.get().pose, vmt.get().timestamp, vmt.get().stdDevs);
        }
    }

    /**
     * YALL Mt1 std
     * @param poseEstimate
     * @param swerve
     * @return
     */
    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag1(
            PoseEstimate poseEstimate, CommandSwerveDrivetrain swerve) {
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
     * YALL std mt2
     * @param poseEstimate
     * @param swerve
     * @return
     */
    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(
            PoseEstimate poseEstimate, CommandSwerveDrivetrain swerve) {
        if (!validPoseEstimate(poseEstimate)) return Optional.empty();
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

        Optional<PoseEstimate> turret = turretPoseEstimatorMT2.getPoseEstimate();
        Optional<PoseEstimate> side = sidePoseEstimatorMT2.getPoseEstimate();
        if (turret.isEmpty() || side.isEmpty()) {
            return;
        }
        Pose3d turretPose = turret.get().pose;
        Pose3d sidePose = side.get().pose;

        Pose3d diff = new Pose3d().plus(sidePose.minus(turretPose));
        Logger.logPose3dAsDoubleArray(LimelightConstants.subsystemName, "transformBetweenCameras", diff);
        diffPublisher.get().set(diff);
    }

    private boolean validPoseEstimate(PoseEstimate pe) {
        if (pe == null
                || !pe.hasData
                || pe.rawFiducials == null
                || pe.rawFiducials.length == 0) return false;
        return true;
    }
}
