package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.Logger;
import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
    public static record VisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    private final Optional<StructPublisher<Pose2d>> MT2TurretPublisher, MT2SidePublisher;
    private String turret, side;
    private boolean throttle;

    public LimelightSubsystem(
            String turretLimelight, String sideLimelight, Pose3d cameraOffsetTurret, Pose3d cameraOffsetSide) {
        turret = "limelight-" + turretLimelight;
        side = "limelight-" + sideLimelight;

        // Configure limelights
        initLimelight(turret, cameraOffsetTurret);
        initLimelight(side, cameraOffsetSide);

        throttle = false;

        // Logging for MT2
        MT2TurretPublisher = Logger.makeStructPublisher(getName(), "MT2" + turretLimelight + "Pose", Pose2d.struct);
        MT2SidePublisher = Logger.makeStructPublisher(getName(), "MT2" + sideLimelight + "Pose", Pose2d.struct);
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
        setRobotOrientationSwerve(); // for external imu modes

        // Log difference in MT2 measurements between both camaras
        // logMT2diff(); // not using MT2

        // log the individual poses of MT2
        logMT2Turret();
        logMT2Side();

        // Now update swerve with two vision measurements
        multiple();

        // logVisionTargets(turretLimelight, visionTargetsTurret);
        // logVisionTargets(sideLimelight, visionTargetsSide);
    }

    public void slowPeriodic() {}

    public void verySlowPeriodic() {}

    private void initLimelight(String name, Pose3d offset) {
        LimelightHelpers.setRewindEnabled(name, true);

        LimelightHelpers.SetIMUMode(name, LimelightConstants.imuMode);
        LimelightHelpers.setStreamMode_Standard(name);

        setLimelightOffset(name, offset);

        LimelightHelpers.SetIMUAssistAlpha(name, LimelightConstants.imuAssistAlpha);
        LimelightHelpers.setPipelineIndex(name, 1); // idk about this

        LimelightHelpers.Flush();
    }

    private void setLimelightOffset(String name, Pose3d offset) {
        double[] offsetArray = new double[6];
        offsetArray[0] = offset.getTranslation().getX();
        offsetArray[1] = offset.getTranslation().getY();
        offsetArray[2] = offset.getTranslation().getZ();
        offsetArray[3] = Units.radiansToDegrees(offset.getRotation().getX());
        offsetArray[4] = Units.radiansToDegrees(offset.getRotation().getY());
        offsetArray[5] = Units.radiansToDegrees(offset.getRotation().getZ());

        LimelightHelpers.setCameraPose_RobotSpace(
                name, offsetArray[0], offsetArray[1], offsetArray[2], offsetArray[3], offsetArray[4], offsetArray[5]);
    }

    /**
     *
     * When throttled, throttle & use viewfinder pipeline,
     * Unthrtottled, no throttle & use Apriltag pipeline
     * @param isEnabled
     */
    public void setThrottle(boolean throttle) {
        if (throttle) {
            LimelightHelpers.SetThrottle(turret, 200);
            LimelightHelpers.SetThrottle(side, 200);

            LimelightHelpers.setPipelineIndex(turret, LimelightConstants.throttlePipelineIndex);
            LimelightHelpers.setPipelineIndex(side, LimelightConstants.throttlePipelineIndex);

            this.throttle = true;
        } else {
            LimelightHelpers.SetThrottle(turret, 0);
            LimelightHelpers.SetThrottle(side, 0);

            LimelightHelpers.setPipelineIndex(turret, LimelightConstants.aprilTagPipelineIndex);
            LimelightHelpers.setPipelineIndex(side, LimelightConstants.aprilTagPipelineIndex);

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
        setRobotOrientationAbsolute(
                RobotContainer.getInstance().swerve.getRotation3d().getMeasureZ());
        // LimelightHelpers.SetRobotOrientation(
        //         turret, RobotContainer.getInstance().swerve.getRotation3d().getZ(), 0, 0, 0, 0, 0);
    }

    private void setRobotOrientationAbsolute(Angle yaw) {
        LimelightHelpers.SetRobotOrientation(turret, yaw.in(Degree), 0, 0, 0, 0, 0);
        LimelightHelpers.Flush();
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
        }
    }
    /**
     * Seeds from swerve, then sets mode to imuMode set in Constants
     */
    public void seedSwerve() {
        CommandSwerveDrivetrain swerve = RobotContainer.getInstance().swerve;
        seedBothAbsolute(swerve.getStateCopy().Pose.getRotation().getMeasure());
    }

    /**
     * Seeds both parameters based on a known absolute angle, then sets the imuMode
     * @param imuMode
     */
    public void seedBothAbsolute(Angle yaw) {
        LimelightHelpers.SetIMUMode(turret, 1);
        LimelightHelpers.SetIMUMode(side, 1);
        LimelightHelpers.Flush();

        setRobotOrientationAbsolute(yaw);

        LimelightHelpers.SetIMUMode(turret, LimelightConstants.imuMode);
        LimelightHelpers.SetIMUMode(side, LimelightConstants.imuMode);
        LimelightHelpers.Flush();
    }

    /**
     * YALL getVisionMeasurement for Turret Camera
     * NOTE:: REMOVED MAX DISTANCE!!!!
     * @param swerve
     * @param useMegaTag2
     * @return
     */
    private Optional<VisionMeasurement> getVisionMeasurementTurret(
            CommandSwerveDrivetrain swerve, boolean useMegaTag2) {
        Optional<Matrix<N3, N1>> stdDevs;
        PoseEstimate poseEstimate;
        if (useMegaTag2) {
            // MT2 Code
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(turret);

            if (poseEstimate.tagCount == 0) return Optional.empty();
            if (poseEstimate.pose.equals(new Pose2d())) return Optional.empty(); // I don't like this

            stdDevs = calculateStdDevsMegaTag2(poseEstimate, swerve);
        } else {
            // MT1 Code
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(turret);

            if (poseEstimate.tagCount == 0) return Optional.empty();
            if (poseEstimate.pose.equals(new Pose2d())) return Optional.empty(); // I don't like this

            stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
        }
        if (stdDevs.isEmpty()) return Optional.empty();
        return Optional.of(new VisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, stdDevs.get()));
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
        PoseEstimate poseEstimate;
        if (useMegaTag2) {
            // MT2 Code
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(side);

            if (poseEstimate.tagCount == 0) return Optional.empty();
            if (poseEstimate.pose.equals(new Pose2d())) return Optional.empty(); // I don't like this

            stdDevs = calculateStdDevsMegaTag2(poseEstimate, swerve);
        } else {
            // MT1 Code
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(side);

            if (poseEstimate.tagCount == 0) return Optional.empty();
            if (poseEstimate.pose.equals(new Pose2d())) return Optional.empty(); // I don't like this

            stdDevs = calculateStdDevsMegaTag1(poseEstimate, swerve);
        }
        if (stdDevs.isEmpty()) return Optional.empty();
        return Optional.of(new VisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, stdDevs.get()));
    }

    /**
     * Gets vision measurements, then adds them to swerve
     */
    private void multiple() {
        CommandSwerveDrivetrain swerve = RobotContainer.getInstance().swerve;
        Optional<VisionMeasurement> vmt = getVisionMeasurementTurret(swerve, true);
        Optional<VisionMeasurement> vms = getVisionMeasurementSide(swerve, true);
        if (!vmt.isEmpty()) {
            swerve.addVisionMeasurement(vmt.get().pose, vmt.get().timestamp, vmt.get().stdDevs);
        }
        if (!vms.isEmpty()) {
            swerve.addVisionMeasurement(vms.get().pose, vms.get().timestamp, vms.get().stdDevs);
        }
    }

    /**
     * Gets vision measurements, then adds them to swerve
     */
    private void multipleMT1() {
        CommandSwerveDrivetrain swerve = RobotContainer.getInstance().swerve;
        Optional<VisionMeasurement> vmt = getVisionMeasurementTurret(swerve, false);
        Optional<VisionMeasurement> vms = getVisionMeasurementSide(swerve, false);
        if (vmt.isPresent()) {
            swerve.addVisionMeasurement(vmt.get().pose, vmt.get().timestamp, vmt.get().stdDevs);
        }
        if (vms.isPresent()) {
            swerve.addVisionMeasurement(vms.get().pose, vms.get().timestamp, vms.get().stdDevs);
        }
    }

    /**
     * Check if pose estimate works first
     * YALL MT1 std
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
            var singleTag = poseEstimate.rawFiducials[0];

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
     * Check if poseEstimate is valid first
     * @param poseEstimate
     * @param swerve
     * @return
     */
    private Optional<Matrix<N3, N1>> calculateStdDevsMegaTag2(
            PoseEstimate poseEstimate, CommandSwerveDrivetrain swerve) {
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
    public void setIMUModeBoth(int imuMode) {
        LimelightHelpers.SetIMUMode(turret, imuMode);
        LimelightHelpers.SetIMUMode(side, imuMode);
        LimelightHelpers.Flush();
    }

    public void setPipelineTurret(short pipe) {
        LimelightHelpers.setPipelineIndex(turret, pipe);
        LimelightHelpers.Flush();
    }

    public void setPipelineSide(short pipe) {
        LimelightHelpers.setPipelineIndex(side, pipe);
        LimelightHelpers.Flush();
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
     * Log the pose we got from MT2 on the turret
     */
    private void logMT2Turret() {
        if (MT2TurretPublisher.isEmpty()) return;

        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(turret);
        if (estimate.tagCount == 0) return;
        if (estimate.pose.equals(new Pose2d())) return;

        MT2TurretPublisher.get().set(estimate.pose);
    }

    /**
     * Log the pose we got from MT2 on the side
     */
    private void logMT2Side() {
        if (MT2SidePublisher.isEmpty()) return;

        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(side);
        if (estimate.tagCount == 0) return;
        if (estimate.pose.equals(new Pose2d())) return;

        MT2SidePublisher.get().set(estimate.pose);
    }
}
