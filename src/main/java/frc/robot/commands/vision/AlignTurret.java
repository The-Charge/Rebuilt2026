package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// commands extend the "Command" class in order to label them as commands
// make sure to descriptively name your commands
// ex. If you are making a command to spin up the indexer, then "SpinUp" is not a good name, as it doesn't make it clear
//     that this command is for the indexer specifically. Something along the lines of "SpinUpIndexer" could be better.
public class AlignTurret extends Command {

    // this is just a reference to the subsystem that was passed in
    // we need this so that we can use the subsystem in the other functions such as initialize and isFinished
    private final LimelightSubsystem lsub;
    private final TurretSubsystem tsub;
    private final SwerveSubsystem ssub;
    private Pose2d poseEstimate;
    private boolean isRed;

    AnalogGyro real = new AnalogGyro(0);
    AnalogGyroSim angle = new AnalogGyroSim(real);

    public AlignTurret(LimelightSubsystem lsub, TurretSubsystem tsub, SwerveSubsystem ssub) {
        this.lsub = lsub;
        this.tsub = tsub;
        this.ssub = ssub;

        addRequirements(tsub);
    }

    @Override
    public void initialize() {

        // initialize runs only once, that being when the command starts

        // LSUB GET POSE, set into pose
        isRed = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }

    @Override
    public void execute() {
        // execute is a lot like "periodic" for subsystems
        // execute will run in a loop until the command ends

        // Get Detection (safe)
        if (lsub.getRawPosition().isEmpty()) {
            SmartDashboard.putString("AprilTagFound", "No apriltag :("); // replace later
            poseEstimate = ssub.getPosition();
        } else {
            SmartDashboard.putString("AprilTagFound", "Found Apriltag :)");
            poseEstimate = lsub.getRawPosition().get();
        }

        // Get Pose2d that points from robot to hub (hub vector - robot vector)
        Transform2d robotToHub;
        if (isRed) robotToHub = FieldConstants.redHubPos.minus(poseEstimate);
        else robotToHub = FieldConstants.blueHubPos.minus(poseEstimate);

        // Set turret angle to robotToHub vector
        Rotation2d rotationToHub = new Rotation2d(robotToHub.getX(), robotToHub.getY());

        tsub.setTurretAngle(rotationToHub);

        angle.setAngle(rotationToHub.getDegrees() + 1);
        SmartDashboard.putData(real);
    }

    // private AprilTagFieldLayout a;

    @Override
    public void end(boolean interrupted) {
        // end is called only once, when the command ends and is exiting
        // the "interrupted" argument is true when the command was forcibly ended and is false when the command ended
        //     'of its own will'
    }

    @Override
    public boolean isFinished() {
        // isFinished determines when the command willing ends
        // return true to end the command
        // return false to keep the command running for another 'frame'
        // return guideSub.getExampleMotorRPM() >= 90;
        return false;
    }
}
