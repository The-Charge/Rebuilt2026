package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class InactiveLED extends Command {

    private final LEDSubsystem led;

    public InactiveLED(LEDSubsystem ledSub) {
        led = ledSub;
        addRequirements(led);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        led.solidColor(Color.kWhite);
    }

    @Override
    public void end(boolean interrupted) {
        led.off();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
