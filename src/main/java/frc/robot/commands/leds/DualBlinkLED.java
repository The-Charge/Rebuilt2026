package frc.robot.commands.leds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;

public class DualBlinkLED extends Command {

    private final LEDSubsystem led;
    private final Color color1, color2;
    private final Optional<Time> expiration;

    private Timer blinkTimer;
    private int blinkIncrement;
    private int lastBlinkIncrement;
    private Timer endTimer;

    public DualBlinkLED(LEDSubsystem ledSub, Color col1, Color col2) {
        led = ledSub;
        color1 = col1;
        color2 = col2;
        expiration = Optional.empty();

        addRequirements(led);
    }

    public DualBlinkLED(LEDSubsystem ledSub, Color col1, Color col2, Time duration) {
        led = ledSub;
        color1 = col1;
        color2 = col2;
        expiration = Optional.of(duration);

        addRequirements(led);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        blinkTimer = new Timer();
        blinkTimer.start();
        blinkIncrement = 0;
        lastBlinkIncrement = -1;

        endTimer = new Timer();
        endTimer.start();
    }

    @Override
    public void execute() {
        while (blinkTimer.advanceIfElapsed(1.0 / 6)) blinkIncrement++;

        if (blinkIncrement != lastBlinkIncrement) {
            if (blinkIncrement % 2 == 0) {
                led.solidColor(color1);
            } else {
                led.solidColor(color2);
            }
        }

        lastBlinkIncrement = blinkIncrement;
    }

    @Override
    public void end(boolean interrupted) {
        led.off();
    }

    @Override
    public boolean isFinished() {
        if (expiration.isEmpty()) return false;

        return endTimer.hasElapsed(expiration.get());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
