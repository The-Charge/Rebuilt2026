package frc.robot.commands.guide;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemGuide;

//commands extend the "Command" class in order to label them as commands
//make sure to descriptively name your commands
//ex. If you are making a command to spin up the indexer, then "SpinUp" is not a good name, as it doesn't make it clear that this command is for the indexer specifically. Something along the lines of "SpinUpIndexer" could be better.
public class CommandGuide extends Command {

    private final SubsystemGuide guideSub;

    public CommandGuide(SubsystemGuide _guideSub) {
        guideSub = _guideSub;
        addRequirements(guideSub);
    }

    @Override
    public void initialize() {
        guideSub.setExampleMotorVelocity(100);
    }

    @Override
    public void execute() {
        //execute is a lot like "periodic" for subsystems
        //execute will run in a loop 
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return guideSub.getExampleMotorRPM() >= 90;
    }
}
