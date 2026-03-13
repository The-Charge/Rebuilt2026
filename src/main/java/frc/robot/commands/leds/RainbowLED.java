package frc.robot.commands.leds;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;

public class RainbowLED extends Command {

    private final LEDSubsystem ledSub;
    private final Optional<Time> expiration;

    private Optional<Timer> timer;

    public RainbowLED(LEDSubsystem LEDSubsystem) {
        ledSub = LEDSubsystem;
        expiration = Optional.empty();
        timer = Optional.empty();

        addRequirements(ledSub);
    }

    public RainbowLED(LEDSubsystem ledSubsystem, Time duration) {
        ledSub = ledSubsystem;
        expiration = Optional.of(duration);
        timer = Optional.empty();

        addRequirements(ledSub);
    }

    @Override
    public void initialize() {
        if (expiration.isPresent()) {
            timer = Optional.of(new Timer());
            timer.get().start();
        }

        ledSub.rainbow(MetersPerSecond.of(4));
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

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
