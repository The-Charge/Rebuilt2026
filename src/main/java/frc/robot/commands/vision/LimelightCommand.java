package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("/MySubsystem/MyPose", Pose2d.struct)
            .publish();

    public LimelightCommand(
            LimelightSubsystem turretLimelightSubsystem,
            LimelightSubsystem sideLimelightSubsystem,
            CommandSwerveDrivetrain swerveSub,
            Supplier<Boolean> enabledSup) {
        turretLimelight = turretLimelightSubsystem;
        sideLimelight = sideLimelightSubsystem;
        swerve = swerveSub;
        enabled = enabledSup;
        addRequirements(turretLimelight, sideLimelight);
    }

    @Override
    public void initialize() {
        enabledLast = !enabled.get();
    }

    @Override
    public void execute() {
        if (enabled.get() && !enabledLast) {
            turretLimelight.setIMUMode(3);
            sideLimelight.setIMUMode(3);
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

        calcMT1diff();

        multiple();

        // Optional<VisionMeasurement> turretVisionEstimateOptional = turretLimelight.getVisionMeasurement(swerve,
        // true);
        // if (turretVisionEstimateOptional.isPresent()) {
        //     VisionMeasurement visionEstimate = turretVisionEstimateOptional.get();
        //     swerve.addVisionMeasurement(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
        // }
    }

    // Use both cameras together (CTRE Swerve; kalman filter)
    @SuppressWarnings("unused")
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
        Optional<Pose2d> turret = turretLimelight.getMegaTag2();
        Optional<Pose2d> side = sideLimelight.getMegaTag2();
        if (turret.isEmpty() || side.isEmpty()) {
            return;
        }
        Pose2d diff = new Pose2d().plus(side.get().minus(turret.get()));
        // Logger.logPose3d(LimelightConstants.subsystemName, "transformBetweenCameras", diff);
        publisher.set(diff);
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
