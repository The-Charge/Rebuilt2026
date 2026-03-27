package frc.robot.commands.leds;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;

public class RainbowLED extends Command {

    private final LEDSubsystem led;
    private final Optional<Time> expiration;

    private Optional<Timer> timer;

    public RainbowLED(LEDSubsystem ledSub) {
        led = ledSub;
        expiration = Optional.empty();
        timer = Optional.empty();

        addRequirements(led);
    }

    public RainbowLED(LEDSubsystem ledSub, Time duration) {
        led = ledSub;
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

        led.rainbow(MetersPerSecond.of(4));
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
