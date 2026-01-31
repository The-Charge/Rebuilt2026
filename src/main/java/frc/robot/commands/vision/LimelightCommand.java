package frc.robot.commands.vision;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem.VisionMeasurement;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightCommand extends Command {
    private final LimelightSubsystem limelightSub;
    private final SwerveSubsystem swerve;

    public LimelightCommand(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSub) {
        limelightSub = limelightSubsystem;
        swerve = swerveSub;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Optional<VisionMeasurement> visionEstimateOptional = limelightSub.getVisionMeasurement(swerve);
        if (visionEstimateOptional.isEmpty()) return;
        VisionMeasurement visionEstimate = visionEstimateOptional.get();
        swerve.addvisionmeasuremant(visionEstimate.pose(), visionEstimate.timestamp(), visionEstimate.stdDevs());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
