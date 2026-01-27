package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.TurretSubsystem;

// commands extend the "Command" class in order to label them as commands
// make sure to descriptively name your commands
// ex. If you are making a command to spin up the indexer, then "SpinUp" is not a good name, as it doesn't make it clear
//     that this command is for the indexer specifically. Something along the lines of "SpinUpIndexer" could be better.
public class AlignTurret extends Command {

    // this is just a reference to the subsystem that was passed in
    // we need this so that we can use the subsystem in the other functions such as initialize and isFinished
    private final LimelightSub lsub;
    private final TurretSubsystem tsub;
    private Pose2d poseEstimate;
    private boolean isRed;

    private Pose2d robotToHub;
    private Rotation2d rotationToHub;

    public AlignTurret(LimelightSub lsub, TurretSubsystem tsub) {
        this.lsub = lsub;
        this.tsub = tsub;

        addRequirements(lsub, tsub);
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
            return;
        } else {
            SmartDashboard.putString("AprilTagFound", "Found Apriltag :)");
        }
        this.poseEstimate = lsub.getRawPosition().get();

        // Get Pose2d that points from robot to hub (hub vector - robot vector)
        if (isRed)
            robotToHub = new Pose2d(FieldConstants.redHubPos.minus(poseEstimate).getTranslation(), new Rotation2d());
        else
            robotToHub =
                    new Pose2d(FieldConstants.blueHubPos.minus(poseEstimate).getTranslation(), new Rotation2d());

        // Set turret angle to robotToHub vector
        rotationToHub = new Rotation2d(robotToHub.getX(), robotToHub.getY());

        tsub.setTurretAngle(rotationToHub);
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
