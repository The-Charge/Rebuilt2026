package frc.robot.subsystems;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import frc.robot.utils.Logger;
import java.util.Optional;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;

    private AddressableLEDBuffer buffer;
    private Optional<LEDPattern> pattern;

    public LEDSubsystem() {

        buffer = new AddressableLEDBuffer(LEDConstants.ledCount);

        led = new AddressableLED(LEDConstants.port);
        led.setLength(buffer.getLength());
        led.setColorOrder(LEDConstants.colorOrder);

        led.setData(buffer);
        led.start();

        pattern = Optional.empty();
    }

    @Override
    public String getName() {
        return LEDConstants.subsystemName;
    }

    @Override
    public void periodic() {
        Logger.logSubsystem(getName(), this);
        Logger.logString(
                getName(),
                "currentPattern",
                pattern.map((val) -> val.getClass().getTypeName()).orElse("None"));
    }

    public void slowPeriodic() {
        if (pattern != null && pattern.isPresent()) {
            pattern.get().applyTo(buffer);
        }

        led.setData(buffer);
    }

    public void verySlowPeriodic() {}

    public void solidColor(Color col) {
        pattern = Optional.of(LEDPattern.solid(col));
    }

    public void blink(Color color, Time onTime) {
        LEDPattern blinkingColor = LEDPattern.solid(color);
        pattern = Optional.of(blinkingColor.blink(onTime));
    }

    public void off() {
        pattern = Optional.of(LEDPattern.solid(Color.kBlack));
    }

    public void rainbow(LinearVelocity scrollSpeed) {
        LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(scrollSpeed, LEDConstants.kLedSpacing);

        pattern = Optional.of(scrollingRainbow);
    }

    public void breathe(Color col, Time cycleTime) {
        LEDPattern base = LEDPattern.solid(col);
        pattern = Optional.of(base.breathe(cycleTime));
    }
}
