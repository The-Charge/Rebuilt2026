package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

// Set Speed on climb manually
public class ManualSpool extends Command {

    private final ClimbSubsystem climb;
    private final double perc;

    public ManualSpool(ClimbSubsystem climbSub, double dutyCycle) {
        climb = climbSub;
        perc = dutyCycle;

        addRequirements(climb);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        climb.dutyCycle(perc);
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
