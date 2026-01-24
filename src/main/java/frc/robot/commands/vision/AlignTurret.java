package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SubsystemGuide;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// commands extend the "Command" class in order to label them as commands
// make sure to descriptively name your commands
// ex. If you are making a command to spin up the indexer, then "SpinUp" is not a good name, as it doesn't make it clear
//     that this command is for the indexer specifically. Something along the lines of "SpinUpIndexer" could be better.
public class AlignTurret extends Command {

    // this is just a reference to the subsystem that was passed in
    // we need this so that we can use the subsystem in the other functions such as initialize and isFinished
    private final LimelightSubsystem lsub;
    private final TurretSubsystem tsub;
    private PoseEstimate poseEstimate;
    private PIDController pid;
    public AlignTurret(LimelightSubsystem lsub, TurretSubsystem tsub) {
        this.lsub = lsub;
        this.tsub = tsub;

        pid = new PIDController(TurretConstants.shooterPID.getP(), TurretConstants.shooterPID.getI(), TurretConstants.shooterPID.getD());
        pid.setSetpoint(TurretConstants.shooterAcceptableAngle);


        addRequirements(lsub, tsub);
    }
    @Override
    public void initialize() {
        // initialize runs only once, that being when the command starts

        // LSUB GET POSE, set into pose
        this.poseEstimate = null;
    }

    @Override
    public void execute() {
        // execute is a lot like "periodic" for subsystems
        // execute will run in a loop until the command ends

    }

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
