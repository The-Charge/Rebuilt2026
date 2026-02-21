package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class ActiveAtFZoneLED extends Command {

    private final LEDSubsystem ledSub;

    public ActiveAtFZoneLED(LEDSubsystem LEDSubsystem) {
        ledSub = LEDSubsystem;
        addRequirements(ledSub);
    }

    @Override
    public void initialize() {
        ledSub.solidColor(Color.kWhite);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        // ledSub.turnOff();
    }

    @Override
    public boolean isFinished() {
        return false; // don't end unless interrupted
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
