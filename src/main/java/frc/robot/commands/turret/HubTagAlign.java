package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.Optional;

public class HubTagAlign extends Command {

    private final TurretSubsystem tsub;
    private final SwerveSubsystem ssub;
    private final LimelightSubsystem lsub;
    // private Pose2d poseEstimate;
    private boolean isRed;

    public HubTagAlign(TurretSubsystem tsub, SwerveSubsystem ssub, LimelightSubsystem lsub) {
        this.tsub = tsub;
        this.ssub = ssub;
        this.lsub = lsub;

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
        Optional<Pose3d> position = lsub.getTransformToTag(20); // change based on which alliance

        // Gets actual pose (safe)
        if (position.isEmpty()) return;
        Pose3d pose = position.get();

        // Set turret angle to robotToHub vector
        Rotation2d rotationToHub = new Rotation2d(pose.getX(), pose.getY());

        SmartDashboard.putNumber("rotation given to turret", rotationToHub.getRadians());
        tsub.setTurretAngle(rotationToHub.plus(new Rotation2d(Math.PI)));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
