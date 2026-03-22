package frc.robot.commands.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;

public class BlinkLED extends Command {

    private final LEDSubsystem led;
    private final Color color;
    private final Optional<Time> expiration;

    private Optional<Timer> timer;

    public BlinkLED(LEDSubsystem ledSub, Color col) {
        led = ledSub;
        color = col;
        expiration = Optional.empty();
        timer = Optional.empty();

        addRequirements(led);
    }

    public BlinkLED(LEDSubsystem ledSub, Color col, Time duration) {
        led = ledSub;
        color = col;
        expiration = Optional.of(duration);
        timer = Optional.empty();

        addRequirements(led);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        if (expiration.isPresent()) {
            timer = Optional.of(new Timer());
            timer.get().start();
        }

        led.blink(color, Seconds.of(1.0 / 3));
    }

    @Override
    public void end(boolean interrupted) {
        led.off();
    }

    @Override
    public boolean isFinished() {
        if (timer.isEmpty()) return false;

        return timer.get().hasElapsed(expiration.orElse(Seconds.of(0)));
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
