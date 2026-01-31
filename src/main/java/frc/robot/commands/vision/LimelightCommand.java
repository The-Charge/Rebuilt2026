package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
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
        var estim = limelightSub.getVisionMeasurement(swerve);
        if (estim.isEmpty()) return;
        var exi = estim.get();
        swerve.addvisionmeasuremant(exi.pose(), exi.timestamp(), exi.stdDevs());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
