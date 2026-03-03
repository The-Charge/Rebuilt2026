package frc.robot.commands.leds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import java.util.Optional;

public class DualBlinkLED extends Command {

    private final LEDSubsystem ledSub;
    private final Color color1, color2;
    private final Optional<Time> expiration;

    private Timer blinkTimer;
    private int blinkIncrement;
    private int lastBlinkIncrement;
    private Timer endTimer;

    public DualBlinkLED(LEDSubsystem LEDSubsystem, Color col1, Color col2) {
        ledSub = LEDSubsystem;
        color1 = col1;
        color2 = col2;
        expiration = Optional.empty();

        addRequirements(ledSub);
    }

    public DualBlinkLED(LEDSubsystem ledSubsystem, Color col1, Color col2, Time duration) {
        ledSub = ledSubsystem;
        color1 = col1;
        color2 = col2;
        expiration = Optional.of(duration);

        addRequirements(ledSub);
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
                ledSub.solidColor(color1);
            } else {
                ledSub.solidColor(color2);
            }
        }

        lastBlinkIncrement = blinkIncrement;
    }

    @Override
    public void end(boolean interrupted) {
        ledSub.off();
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
