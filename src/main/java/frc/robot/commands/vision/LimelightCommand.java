package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.VisionMeasurement;
import frc.robot.utils.Logger;
import java.util.Optional;
import java.util.function.Supplier;

public class LimelightCommand extends Command {
    private final LimelightSubsystem turretLimelight;
    private final LimelightSubsystem sideLimelight;
    private final CommandSwerveDrivetrain swerve;
    private final Supplier<Boolean> enabled;
    private boolean enabledLast;
    private boolean isSeeded;

    private final Optional<StructPublisher<Pose3d>> diffPublisher;
    private final Optional<StructPublisher<Pose2d>> turretPub =
            Logger.makeStructPublisher(LimelightConstants.subsystemName, "turretPose", Pose2d.struct);
    private final Optional<StructPublisher<Pose2d>> sidePub =
            Logger.makeStructPublisher(LimelightConstants.subsystemName, "sidePose", Pose2d.struct);

    public LimelightCommand(
            LimelightSubsystem turretLimelightSubsystem,
            LimelightSubsystem sideLimelightSubsystem,
            CommandSwerveDrivetrain swerveSub,
            Supplier<Boolean> enabledSup) {
        turretLimelight = turretLimelightSubsystem;
        sideLimelight = sideLimelightSubsystem;
        swerve = swerveSub;
        enabled = enabledSup;
        isSeeded = false;
        addRequirements(turretLimelight, sideLimelight);

        diffPublisher = Logger.makeStructPublisher(getName(), "poseDeviation", Pose3d.struct);
    }

    @Override
    public void initialize() {
        enabledLast = !enabled.get();
    }

    @Override
    public void execute() {
        if (enabled.get() && !enabledLast) {
            // turretLimelight.setIMUMode(3);
            // sideLimelight.setIMUMode(3);
            turretLimelight.setThrottle(false);
            sideLimelight.setThrottle(false);
            turretLimelight.setPipeline(1);
            sideLimelight.setPipeline(1);
            Logger.logBool(LimelightConstants.subsystemName, "enabled", true);
            enabledLast = true;
        }
        if (!enabled.get() && enabledLast) {
            // turretLimelight.setIMUMode(1);
            // sideLimelight.setIMUMode(1);
            turretLimelight.setThrottle(true);
            sideLimelight.setThrottle(true);
            turretLimelight.setPipeline(0);
            sideLimelight.setPipeline(0);
            Logger.logBool(LimelightConstants.subsystemName, "enabled", false);
            enabledLast = false;
        }

        if (!isSeeded) {
            seed();
        }

        setRobotOrientation();

        calcMT1diff();

        multiple();

        // Optional<VisionMeasurement> turretVisionEstimateOptional = turretLimelight.getVisionMeasurement(swerve,
        // true);
        // if (turretVisionEstimateOptional.isPresent()) {
        //     VisionMeasurement visionEstimate = turretVisionEstimateOptional.get();
        //     swerve.addVisionMeasurement(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
        // }
    }

    /**
     * Seed from MT1; chooses best MT1 from the two cameras,
     * sets that as the seed for both cameras
     */
    private void seed() {
        Optional<LimelightSubsystem.VisionMeasurement> MT1turret = turretLimelight.getVisionMeasurement(swerve, false);
        Optional<LimelightSubsystem.VisionMeasurement> MT1side = sideLimelight.getVisionMeasurement(swerve, false);
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
            turretLimelight.seedInternalIMU(rots);
            sideLimelight.seedInternalIMU(rots);
            isSeeded = true;
        }
    }
    /**
     * Seed from PathPlanner pose at beginning of auton
     * aka seed from Swerve
     */
    public void seedFromIMU() {
        Angle rot = swerve.getStateCopy().Pose.getRotation().getMeasure();
        turretLimelight.seedInternalIMU(rot);
        sideLimelight.seedInternalIMU(rot);
        isSeeded = true;
    }
    /**
     * Seed from absolute driver position
     */
    public void seedFromAbsolute(double deg) {
        Angle rot = (Degrees.of(deg));
        turretLimelight.seedInternalIMU(rot);
        sideLimelight.seedInternalIMU(rot);
        isSeeded = true;
    }

    private void setRobotOrientation() {
        Optional<LimelightSubsystem.VisionMeasurement> MT1turret = turretLimelight.getVisionMeasurement(swerve, false);
        Optional<LimelightSubsystem.VisionMeasurement> MT1side = sideLimelight.getVisionMeasurement(swerve, false);

        if (MT1turret.isPresent() && turretPub.isPresent()) {
            turretPub.get().set(MT1turret.get().pose());
        }
        if (MT1side.isPresent() && sidePub.isPresent()) {
            sidePub.get().set(MT1side.get().pose());
        }

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

            Logger.logDouble(LimelightConstants.subsystemName, "Angle of Bot", rots.in(Degrees));
            turretLimelight.setRobotOrientation(rots);
            sideLimelight.setRobotOrientation(rots);
        }
    }

    // Use both cameras together (CTRE Swerve; kalman filter)
    private void multiple() {
        Optional<VisionMeasurement> turretVisionEstimateOptional = turretLimelight.getVisionMeasurement(swerve);
        Optional<VisionMeasurement> sideVisionEstimateOptional = sideLimelight.getVisionMeasurement(swerve);

        if (turretVisionEstimateOptional.isPresent()) {
            VisionMeasurement visionEstimate = turretVisionEstimateOptional.get();
            swerve.addVisionMeasurement(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
        }

        if (sideVisionEstimateOptional.isPresent()) {
            VisionMeasurement visionEstimate = sideVisionEstimateOptional.get();
            swerve.addVisionMeasurement(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
        }
    }

    // Strictly prefers Turret limelight
    @SuppressWarnings("unused")
    private void oneAtATime() {
        Optional<VisionMeasurement> turretVisionEstimateOptional = turretLimelight.getVisionMeasurement(swerve);
        Optional<VisionMeasurement> sideVisionEstimateOptional = sideLimelight.getVisionMeasurement(swerve);
        VisionMeasurement visionEstimate;
        if (turretVisionEstimateOptional.isEmpty()) {
            if (sideVisionEstimateOptional.isEmpty()) {
                return;
            } else {
                visionEstimate = sideVisionEstimateOptional.get();
            }
        } else {
            visionEstimate = turretVisionEstimateOptional.get();
        }
        swerve.addVisionMeasurement(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
    }

    // tries to find which measurement is best
    @SuppressWarnings("unused")
    private void preferential() {
        Optional<VisionMeasurement> turretVisionEstimateOptional = turretLimelight.getVisionMeasurement(swerve);
        Optional<VisionMeasurement> sideVisionEstimateOptional = sideLimelight.getVisionMeasurement(swerve);
        VisionMeasurement visionEstimate;
        if (turretVisionEstimateOptional.isEmpty()) {
            if (sideVisionEstimateOptional.isEmpty()) return;

            visionEstimate = sideVisionEstimateOptional.get();
        } else {
            if (sideVisionEstimateOptional.isEmpty()) {
                visionEstimate = turretVisionEstimateOptional.get();
            } else {
                if (turretVisionEstimateOptional.get().stdDevs().get(0, 0)
                        > sideVisionEstimateOptional.get().stdDevs().get(0, 0)) {
                    visionEstimate = sideVisionEstimateOptional.get();
                } else {
                    visionEstimate = turretVisionEstimateOptional.get();
                }
            }
        }
        swerve.addVisionMeasurement(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
    }

    private void calcMT1diff() {
        if (diffPublisher.isEmpty()) return;

        Optional<Pose3d> turret = turretLimelight.getMegaTag1();
        Optional<Pose3d> side = sideLimelight.getMegaTag1();
        if (turret.isEmpty() || side.isEmpty()) {
            return;
        }
        Pose3d diff = new Pose3d().plus(side.get().minus(turret.get()));
        // Logger.logPose3dAsDoubleArray(LimelightConstants.subsystemName, "transformBetweenCameras", diff);
        diffPublisher.get().set(diff);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
