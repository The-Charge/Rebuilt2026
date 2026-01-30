package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClimbSubsystem;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.SubsystemGuide;


public class ClimbDown extends Command {

  
    private final ClimbSubsystem climbSubsystem;

    public ClimbDown(ClimbSubsystem climb) {
        climbSubsystem = climb;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        
        climbSubsystem.setClimbMotorPosition(ClimberConstants.PositionDown);
    }


    @Override
    public void end(boolean interrupted) {
        // end is called only once, when the command ends and is exiting
        // the "interrupted" argument is true when the command was forcibly ended and is false when the command ended
        //     'of its own will'
        climbSubsystem.stop();

        
    }

    @Override
    public boolean isFinished() {
        // isFinished determines when the command willing ends
        // return true to end the command
        // return false to keep the command running for another 'frame'
        return climbSubsystem.getPosition() <= 90;
    }
}
