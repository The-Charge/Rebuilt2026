package frc.robot;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.SubsystemGuide;

public class ClimbCommand extends Command {
    private final ClimbSubsystem climbSubsystem;


    public ClimbCommand(ClimbSubsystem climb) {
        climbSubsystem = climb;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        // initialize runs only once, that being when the command starts
         
        climbSubsystem.setClimbMotorPosition(ClimberConstants.Position);

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
         final double robotPos;

        robotPos = climbSubsystem.getPosition();
        return climbSubsystem.getPosition() >= 90;
    }
}