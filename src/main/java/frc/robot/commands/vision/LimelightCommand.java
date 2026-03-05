package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.VisionMeasurement;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Logger;
import java.util.Optional;
import java.util.function.Supplier;

public class LimelightCommand extends Command {
    private final LimelightSubsystem turretLimelight;
    private final LimelightSubsystem sideLimelight;
    private final SwerveSubsystem swerve;
    private final Supplier<Boolean> enabled;

    public LimelightCommand(
            LimelightSubsystem turretLimelightSubsystem,
            LimelightSubsystem sideLimelightSubsystem,
            SwerveSubsystem swerveSub,
            Supplier<Boolean> enabledSup) {
        turretLimelight = turretLimelightSubsystem;
        sideLimelight = sideLimelightSubsystem;
        swerve = swerveSub;
        enabled = enabledSup;
    }

    @Override
    public void initialize() {
        turretLimelight.setIMUMode(1);
        sideLimelight.setIMUMode(1);
    }

    @Override
    public void execute() {
        if (enabled.get()) {
            turretLimelight.setIMUMode(3);
            sideLimelight.setIMUMode(3);
            turretLimelight.setThrottle(false);
            sideLimelight.setThrottle(false);
        } else {
            turretLimelight.setIMUMode(1);
            sideLimelight.setIMUMode(1);
            turretLimelight.setThrottle(true);
            sideLimelight.setThrottle(true);
        }

        calcMT1diff();

        preferential();
    }

    // Use both cameras together (CTRE Swerve; kalman filter)
    @SuppressWarnings("unused")
    private void multiple() {
        Optional<VisionMeasurement> turretVisionEstimateOptional = turretLimelight.getVisionMeasurement(swerve);
        Optional<VisionMeasurement> sideVisionEstimateOptional = sideLimelight.getVisionMeasurement(swerve);

        if (turretVisionEstimateOptional.isPresent()) {
            VisionMeasurement visionEstimate = turretVisionEstimateOptional.get();
            swerve.addVisionReading(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
        }

        if (sideVisionEstimateOptional.isPresent()) {
            VisionMeasurement visionEstimate = sideVisionEstimateOptional.get();
            swerve.addVisionReading(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
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
        swerve.addVisionReading(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
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
        swerve.addVisionReading(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
    }

    private void calcMT1diff() {
        Optional<Pose3d> turret = turretLimelight.getMegaTag1();
        Optional<Pose3d> side = sideLimelight.getMegaTag1();
        if (turret.isEmpty() || side.isEmpty()) {
            return;
        }
        Pose3d diff = new Pose3d().plus(side.get().minus(turret.get()));
        Logger.logPose3d(LimelightConstants.subsystemName, "transformBetweenCameras", diff);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
