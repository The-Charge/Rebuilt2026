package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class Idle extends Command {
    LEDSubsystem ledSub;

    public Idle(LEDSubsystem LEDSubsystem) {
        ledSub = LEDSubsystem;
        addRequirements(ledSub);
    }

    @Override
    public void initialize() {
        ledSub.idleLeds();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        // ledSub.turnOff();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
