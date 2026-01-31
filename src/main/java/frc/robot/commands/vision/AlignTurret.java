package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AlignTurret extends Command {

    private final TurretSubsystem tsub;
    private final SwerveSubsystem ssub;
    private Pose2d poseEstimate;
    private boolean isRed;

    public AlignTurret(TurretSubsystem tsub, SwerveSubsystem ssub) {
        this.tsub = tsub;
        this.ssub = ssub;

        addRequirements(tsub);
    }

    @Override
    public void initialize() {
        // LSUB GET POSE, set into pose
        isRed = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    @Override
    public void execute() {
        // Get Detection (safe)
        poseEstimate = ssub.getPose(); // maybe we could get position from limelights instead

        // Get Pose2d that points from robot to hub (hub vector - robot vector)
        Transform2d robotToHub = (isRed ? FieldConstants.redHubPos : FieldConstants.blueHubPos).minus(poseEstimate);

        // Set turret angle to robotToHub vector
        Rotation2d rotationToHub = new Rotation2d(robotToHub.getX(), robotToHub.getY());

        tsub.setTurretAngle(rotationToHub);
    }

    // private AprilTagFieldLayout a;

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
