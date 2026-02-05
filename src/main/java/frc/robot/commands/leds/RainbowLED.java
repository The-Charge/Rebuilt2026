package frc.robot.commands.leds;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class RainbowLED extends Command {

    private final LEDSubsystem ledSub;

    public RainbowLED(LEDSubsystem LEDSubsystem) {
        ledSub = LEDSubsystem;
        addRequirements(ledSub);
    }

    @Override
    public void initialize() {
        ledSub.rainbow(MetersPerSecond.of(1));
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
