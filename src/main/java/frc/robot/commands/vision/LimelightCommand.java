package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.VisionMeasurement;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;

public class LimelightCommand extends Command {
    private final LimelightSubsystem turretLimelight;
    private final LimelightSubsystem sideLimelight;
    private final SwerveSubsystem swerve;

    public LimelightCommand(
            LimelightSubsystem turretLimelightSubsystem,
            LimelightSubsystem sideLimelightSubsystem,
            SwerveSubsystem swerveSub) {
        turretLimelight = turretLimelightSubsystem;
        sideLimelight = sideLimelightSubsystem;
        swerve = swerveSub;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        calcMT1diff();

        multiple();
    }

    // Use both cameras together (CTRE Swerve; kalman filter)
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
    private void preferential() {
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
        Transform3d diff = side.get().minus(turret.get());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
