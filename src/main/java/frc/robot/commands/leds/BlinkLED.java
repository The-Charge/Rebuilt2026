package frc.robot.commands.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;

public class BlinkLED extends Command {

    private final LEDSubsystem ledSub;
    private final Color color;
    private final Optional<Time> expiration;

    private Optional<Timer> timer;

    public BlinkLED(LEDSubsystem LEDSubsystem, Color col) {
        ledSub = LEDSubsystem;
        color = col;
        expiration = Optional.empty();
        timer = Optional.empty();

        addRequirements(ledSub);
    }

    public BlinkLED(LEDSubsystem ledSubsystem, Color col, Time duration) {
        ledSub = ledSubsystem;
        color = col;
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

        ledSub.blink(color, Seconds.of(1.0 / 3));
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        ledSub.off();
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
