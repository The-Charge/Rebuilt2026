package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Logger;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.StreamMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.AprilTagFiducial;
import limelight.results.RawFiducial;

public class LimelightSubsystem extends SubsystemBase {

    public static record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    private final Limelight turretLimelight;
    private final Limelight sideLimelight;

    private final LimelightPoseEstimator turretPoseEstimatorMT2;
    private final LimelightPoseEstimator sidePoseEstimatorMT2;

    private final LimelightPoseEstimator turretPoseEstimatorMT1;
    private final LimelightPoseEstimator sidePoseEstimatorMT1;

    private final Optional<StructArrayPublisher<Pose2d>> visionTargetsTurret;
    private final Optional<StructArrayPublisher<Pose2d>> visionTargetsSide;

    private final Optional<StructPublisher<Pose3d>> diffPublisher;

    private final Optional<StructPublisher<Pose3d>> turretPublisherMT1;
    private final Optional<StructPublisher<Pose3d>> sidePublisherMT1;

    private final Optional<StructPublisher<Pose3d>> turretPublisherMT2;
    private final Optional<StructPublisher<Pose3d>> sidePublisherMT2;

    private boolean isSeeded = false;
    private boolean throttle = false;
    // private boolean isChanged = false;

    public LimelightSubsystem(
            String turretLimelight, String sideLimelight, Pose3d cameraOffsetTurret, Pose3d cameraOffsetSide) {

        // Make limelights
        this.turretLimelight = new Limelight("limelight-" + turretLimelight);
        this.sideLimelight = new Limelight("limelight-" + sideLimelight);

        // Configure limelights
        initLimelights(cameraOffsetTurret, cameraOffsetSide);

        // MT2 Pose Estimators
        this.turretPoseEstimatorMT2 = this.turretLimelight.createPoseEstimator(EstimationMode.MEGATAG2);
        this.sidePoseEstimatorMT2 = this.sideLimelight.createPoseEstimator(EstimationMode.MEGATAG2);

        // MT1 Pose Estimators
        this.turretPoseEstimatorMT1 = this.turretLimelight.createPoseEstimator(EstimationMode.MEGATAG1);
        this.sidePoseEstimatorMT1 = this.sideLimelight.createPoseEstimator(EstimationMode.MEGATAG1);

        // Logging for visiontargets
        visionTargetsTurret = Logger.makeStructArrayPublisher(
                getName(), "limelight-" + turretLimelight + "/targetPoses", Pose2d.struct);
        visionTargetsSide = Logger.makeStructArrayPublisher(
                getName(), "limelight-" + sideLimelight + "/targetPoses", Pose2d.struct);

        // Logging for diff
        diffPublisher = Logger.makeStructPublisher(getName(), "poseDeviation", Pose3d.struct);

        // Logging for MT1 publisher
        turretPublisherMT1 = Logger.makeStructPublisher(getName(), "MT1" + turretLimelight + "Pose", Pose3d.struct);
        sidePublisherMT1 = Logger.makeStructPublisher(getName(), "MT1" + sideLimelight + "Pose", Pose3d.struct);

        // Logging for MT2 Publisher
        turretPublisherMT2 = Logger.makeStructPublisher(getName(), "MT2" + turretLimelight + "Pose", Pose3d.struct);
        sidePublisherMT2 = Logger.makeStructPublisher(getName(), "MT2" + sideLimelight + "Pose", Pose3d.struct);
    }

    @Override
    public String getName() {
        return LimelightConstants.subsystemName;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(getName(), this);
        Logger.logBool(getName(), "throttle", throttle);

        // Seeding is run in Robot.java in autonomousInit(),
        // In RobotContainer: Also run in m_hid on buttonbox, and b on controller (seedFromAbsolute 0 or 180 on
        // blue/red)

        // Update vision with robot rotation information (mode 0)
        // setRobotOrientationSwerve(); // for external imu modes

        // Log difference in MT2 measurements between both camaras
        // logMT2diff(); // not using MT2

        // log the individual poses of MT1
        logTurret();
        logSide();

        // Now update swerve with two vision measurements
        multiple();

        // logVisionTargets(turretLimelight, visionTargetsTurret);
        // logVisionTargets(sideLimelight, visionTargetsSide);
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {}

    private void initLimelights(Pose3d cameraOffsetTurret, Pose3d cameraOffsetSide) {
        LimelightHelpers.setRewindEnabled(turretLimelight.limelightName, true);
        LimelightHelpers.setRewindEnabled(sideLimelight.limelightName, true);
        this.turretLimelight
                .getSettings()
                .withImuMode(LimelightConstants.imuMode) // ehhhhhh
                .withStreamMode(StreamMode.Standard)
                .withCameraOffset(cameraOffsetTurret)
                .withImuAssistAlpha(LimelightConstants.imuAssistAlpha)
                .withPipelineIndex(LimelightConstants.aprilTagPipelineIndex)
                .save();
        this.sideLimelight
                .getSettings()
                .withImuMode(LimelightConstants.imuMode)
                .withStreamMode(StreamMode.Standard)
                .withCameraOffset(cameraOffsetSide)
                .withImuAssistAlpha(LimelightConstants.imuAssistAlpha)
                .withPipelineIndex(LimelightConstants.aprilTagPipelineIndex)
                .save();
    }

    /**
     *
     * When throttled, throttle & use viewfinder pipeline,
     * Unthrtottled, no throttle & use Apriltag pipeline
     * @param isEnabled
     */
    public void setThrottle(boolean throttle) {
        if (throttle) {
            LimelightHelpers.SetThrottle(turretLimelight.limelightName, 200);
            LimelightHelpers.SetThrottle(sideLimelight.limelightName, 200);

            this.turretLimelight
                    .getSettings()
                    .withPipelineIndex(LimelightConstants.throttlePipelineIndex)
                    .save();
            this.sideLimelight
                    .getSettings()
                    .withPipelineIndex(LimelightConstants.throttlePipelineIndex)
                    .save();
            this.throttle = true;
        } else {
            LimelightHelpers.SetThrottle(turretLimelight.limelightName, 0);
            LimelightHelpers.SetThrottle(sideLimelight.limelightName, 0);

            this.turretLimelight
                    .getSettings()
                    .withPipelineIndex(LimelightConstants.aprilTagPipelineIndex)
                    .save();
            this.sideLimelight
                    .getSettings()
                    .withPipelineIndex(LimelightConstants.aprilTagPipelineIndex)
                    .save();
            this.throttle = false;
        }
    }

    /**
     * UNUSED???? fix later ?? (low priority)
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
                .withRobotOrientation(new Orientation3d(new Rotation3d(0, 0, yaw.in(Radians)), angularVelocity))
                .save();
        sideLimelight
                .getSettings()
                .withRobotOrientation(new Orientation3d(new Rotation3d(0, 0, yaw.in(Radians)), angularVelocity))
                .save();
    }

    /**
     * Seeds from MT1, then sets mode to imuMode
     */
    public void seedMT1() {
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
            seedBothAbsolute(rots);
            isSeeded = true;
        }
    }
    /**
     * Seeds from swerve, then sets mode to imuMode set in Constants
     */
    public void seedSwerve() {
        CommandSwerveDrivetrain swerve = RobotContainer.getInstance().swerve;
        seedBothAbsolute(swerve.getStateCopy().Pose.getRotation().getMeasure());
        isSeeded = true;
    }

    /**
     * Seeds both parameters based on a known absolute angle, then sets the imuMode
     * @param imuMode
     */
    public void seedBothAbsolute(Angle yaw) {
        turretLimelight.getSettings().withImuMode(ImuMode.SyncInternalImu).save();
        sideLimelight.getSettings().withImuMode(ImuMode.SyncInternalImu).save();

        setRobotOrientationAbsolute(yaw);

        sideLimelight.getSettings().withImuMode(LimelightConstants.imuMode).save();
        turretLimelight.getSettings().withImuMode(LimelightConstants.imuMode).save();
    }

    public boolean getIsSeeded() {
        return isSeeded;
    }

    /**
     * TODO: The reason why our robot is teleporting ???
     * @param limelight
     * @param targetPublisher
     */
    private void logVisionTargets(Limelight limelight, Optional<StructArrayPublisher<Pose2d>> targetPublisher) {
        Optional<limelight.networktables.LimelightResults> results = limelight.getLatestResults();

        double[] targetIDs;
        Pose2d[] targetPoses;
        if (results.isEmpty()) {
            targetIDs = new double[] {};
            targetPoses = new Pose2d[] {};
        } else {
            AprilTagFiducial[] targetFiducials = results.get().targets_Fiducials;

            targetIDs = new double[targetFiducials.length];
            targetPoses = new Pose2d[targetFiducials.length];

            for (int i = 0; i < targetFiducials.length; i++) {
                targetIDs[i] = targetFiducials[i].fiducialID;
                targetPoses[i] = LimelightConstants.tagLayout
                        .getTagPose((int) targetFiducials[i].fiducialID)
                        .map((val) -> val.toPose2d())
                        .orElse(new Pose2d(Double.NaN, Double.NaN, Rotation2d.kZero));
            }
        }

        Logger.logDoubleArray(getName(), limelight.limelightName + "/targetIDs", targetIDs);
        if (targetPublisher.isPresent()) {
            targetPublisher.get().set(targetPoses);
        }
    }
    /**
     * Getters just in case!!!
     * @return
     */
    private Optional<PoseEstimate> getMegaTag1Turret() {
        Optional<PoseEstimate> poseEstimate = turretPoseEstimatorMT1.getPoseEstimate();
        if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
        return poseEstimate;
    }

    /**
     * Getters just in case!!!
     * @return
     */
    private Optional<PoseEstimate> getMegTag1Side() {
        Optional<PoseEstimate> poseEstimate = sidePoseEstimatorMT1.getPoseEstimate();
        if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
        return poseEstimate;
    }

    /**
     * Getters just in case!!!
     * @return
     */
    private Optional<PoseEstimate> getMegaTag2Turret() {
        Optional<PoseEstimate> poseEstimate = turretPoseEstimatorMT2.getPoseEstimate();
        if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
        return poseEstimate;
    }

    /**
     * Getters just in case!!!
     * @return
     */
    private Optional<PoseEstimate> getMegaTag2Side() {
        Optional<PoseEstimate> poseEstimate = sidePoseEstimatorMT2.getPoseEstimate();
        if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
        return poseEstimate;
    }

    /**
     * YALL getVisionMeasurement for Turret Camera
     * NOTE:: REMOVED MAX DISTANCE!!!!
     * kept boolean for useMT2 because might need to seed from MT1 even if using MT2 as active mode
     * @param swerve
     * @param useMegaTag2
     * @return
     */
    private Optional<VisionMeasurement> getVisionMeasurementTurret(
            CommandSwerveDrivetrain swerve, boolean useMT2) {
        Optional<Matrix<N3, N1>> stdDevs;
        Optional<PoseEstimate> poseEstimate;
        if (useMT2) {
            // MT2 Code
            poseEstimate = turretPoseEstimatorMT2.getPoseEstimate();
            if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
            stdDevs = calculateStdDevsMegaTag2(poseEstimate.get(), swerve);
            if (stdDevs.isEmpty()) return Optional.empty();
        } else {
            // MT1 Code
            poseEstimate = turretPoseEstimatorMT1.getPoseEstimate();
            if (poseEstimate.isEmpty()) {
                return Optional.empty();
            }

            stdDevs = calculateStdDevsMegaTag1(poseEstimate.get(), swerve);

            if (stdDevs.isEmpty()) return Optional.empty();
        }
        return Optional.of(new VisionMeasurement(
                poseEstimate.get().pose.toPose2d(), poseEstimate.get().timestampSeconds, stdDevs.get()));
    }

    /**
     * YALL getVisionMeasurement for Side Camera
     * NOTE:: REMOVED MAX DISTANCE!!!!
     * NEED TO MAKE BOOL MEGATAG2 DO SOMETHING!!
     * @param swerve
     * @param useMegaTag2
     * @return
     */
    private Optional<VisionMeasurement> getVisionMeasurementSide(CommandSwerveDrivetrain swerve, boolean useMegaTag2) {
        Optional<Matrix<N3, N1>> stdDevs;
        Optional<PoseEstimate> poseEstimate;
        if (useMegaTag2) {
            // MT2 Code
            poseEstimate = sidePoseEstimatorMT2.getPoseEstimate();
            if (poseEstimate.isEmpty() || !validPoseEstimate(poseEstimate.get())) return Optional.empty();
            stdDevs = calculateStdDevsMegaTag2(poseEstimate.get(), swerve);
            if (stdDevs.isEmpty()) return Optional.empty();
        } else {
            // MT1 Code
            poseEstimate = sidePoseEstimatorMT1.getPoseEstimate();
            if (poseEstimate.isEmpty()) {
                return Optional.empty();
            }

            stdDevs = calculateStdDevsMegaTag1(poseEstimate.get(), swerve);

            if (stdDevs.isEmpty()) return Optional.empty();
        }
        return Optional.of(new VisionMeasurement(
                poseEstimate.get().pose.toPose2d(), poseEstimate.get().timestampSeconds, stdDevs.get()));
    }

    /**
     * Gets vision measurements, then adds them to swerve
     */
    private void multiple() {
        CommandSwerveDrivetrain swerve = RobotContainer.getInstance().swerve;
        Optional<VisionMeasurement> vmt = getVisionMeasurementTurret(swerve, LimelightConstants.activeMode.equals(LimelightConstants.Mode.MT2));
        Optional<VisionMeasurement> vms = getVisionMeasurementSide(swerve, LimelightConstants.activeMode.equals(LimelightConstants.Mode.MT2));
        if (!vmt.isEmpty()) {
            swerve.addVisionMeasurement(vmt.get().pose, vmt.get().timestamp, vmt.get().stdDevs);
        }
        if (!vms.isEmpty()) {
            swerve.addVisionMeasurement(vms.get().pose, vms.get().timestamp, vms.get().stdDevs);
        }
    }

    /**
     * YALL MT1 std
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

        double rotStdDev = LimelightConstants.kRotStdDev; // we want to get the rotation from megatag1

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

        double rotStdDev = LimelightConstants.kRotStdDev / 4; // never trust rotation under any circumstances, but
        // maybe do
        // double rotStdDev = Double.MAX_VALUE;

        return Optional.of(VecBuilder.fill(transStdDev, transStdDev, rotStdDev));
    }

    /**
     * Sets ImuMode of both limelights
     * @param imuMode
     */
    public void setIMUModeBoth(ImuMode imuMode) {
        turretLimelight.getSettings().withImuMode(imuMode).save();
        sideLimelight.getSettings().withImuMode(imuMode).save();
    }

    public void setPipelineTurret(short pipe) {
        turretLimelight.getSettings().withPipelineIndex(pipe).save();
    }

    public void setPipelineSide(short pipe) {
        sideLimelight.getSettings().withPipelineIndex(pipe).save();
    }

    /**
     * FIX REWIND, THROTTLE
     */
    // public void takeRewind() {
    //     LimelightHelpers.triggerRewindCapture(cameraName, 200.0);
    // }
    // public void setThrottle(boolean enabled) {
    // LimelightHelpers.SetThrottle(cameraName, enabled ? 200 : 0);
    // }

    /**
     * Logs difference between MT2 measurements between camaras.
     */
    private void logDiff() {
        // MT2 Case
        if (LimelightConstants.activeMode.equals(LimelightConstants.Mode.MT2)) {
            if (diffPublisher.isEmpty()) return;

            Optional<PoseEstimate> turret = turretPoseEstimatorMT2.getPoseEstimate();
            Optional<PoseEstimate> side = sidePoseEstimatorMT2.getPoseEstimate();
            if (turret.isEmpty()
                    || side.isEmpty()
                    || !validPoseEstimate(turret.get())
                    || !validPoseEstimate(side.get())) return;
            Pose3d turretPose = turret.get().pose;
            Pose3d sidePose = side.get().pose;

            Pose3d diff = new Pose3d().plus(sidePose.minus(turretPose));
            diffPublisher.get().set(diff);
        } else {
            // MT1
            if (diffPublisher.isEmpty()) return;

            Optional<PoseEstimate> turret = turretPoseEstimatorMT1.getPoseEstimate();
            Optional<PoseEstimate> side = sidePoseEstimatorMT1.getPoseEstimate();
            if (turret.isEmpty()
                    || side.isEmpty()
                    || !validPoseEstimate(turret.get())
                    || !validPoseEstimate(side.get())) return;
            Pose3d turretPose = turret.get().pose;
            Pose3d sidePose = side.get().pose;

            Pose3d diff = new Pose3d().plus(sidePose.minus(turretPose));
            diffPublisher.get().set(diff);
        }
    }

    /**
     * Log the pose we got from turret; uses poseEstimators from Constant's activeMode
     */
    private void logTurret() {
        if (LimelightConstants.activeMode == LimelightConstants.Mode.MT1) {
            if (turretPublisherMT1.isEmpty()) return;

            Optional<PoseEstimate> opt = turretPoseEstimatorMT1.getPoseEstimate();
            if (opt.isEmpty()) return;
            PoseEstimate poseEstimate = opt.get();
            if (!validPoseEstimate(poseEstimate)) return;

            turretPublisherMT1.get().set(opt.get().pose);
        } else {
            if (turretPublisherMT2.isEmpty()) return;

            var pose = turretPoseEstimatorMT2.getPoseEstimate();
            if (pose.isEmpty()) return;

            turretPublisherMT2.get().set(pose.get().pose);
        }
    }

    /**
     * Log the pose we got from side
     */
    private void logSide() {
        if (LimelightConstants.activeMode == LimelightConstants.Mode.MT1) {
            if (sidePublisherMT1.isEmpty()) return;

            Optional<PoseEstimate> opt = sidePoseEstimatorMT1.getPoseEstimate();
            if (opt.isEmpty()) return;
            PoseEstimate poseEstimate = opt.get();
            if (!validPoseEstimate(poseEstimate)) return;

            sidePublisherMT1.get().set(opt.get().pose);
        } else {
            if (sidePublisherMT2.isEmpty()) return;

            var pose = sidePoseEstimatorMT2.getPoseEstimate();
            if (pose.isEmpty()) return;

            sidePublisherMT2.get().set(pose.get().pose);
        }
    }

    /**
     * Returns whether given pose estimate is valid. First check if optional is empty!
     * @param pe
     * @return
     */
    private boolean validPoseEstimate(PoseEstimate pe) {
        if (pe == null || !pe.hasData || pe.rawFiducials == null || pe.rawFiducials.length == 0) return false;
        return true;
    }
}
