package frc.robot.commands.guide;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class BlinkLED extends Command {
    LEDSubsystem ledSub;
    Color LEDColor;

    public BlinkLED(LEDSubsystem LEDSubsystem, Color color) {
        ledSub = LEDSubsystem;
        addRequirements(ledSub);

        LEDColor = color;
    }

    @Override
    public void initialize() {
        ledSub.blink(LEDColor);
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
