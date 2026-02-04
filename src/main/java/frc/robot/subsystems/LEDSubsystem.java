package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import java.util.Optional;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private AddressableLEDBuffer buffer;
    private Optional<LEDPattern> pattern;

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.port); // port number
        buffer = new AddressableLEDBuffer(LEDConstants.ledCount); // nuumber of LEDs
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        pattern = Optional.empty();
    }

    @Override
    public void periodic() {
        if (pattern != null && pattern.isPresent()) {
            pattern.get().applyTo(buffer);
        }

        led.setData(buffer);
    }

    public void solidColor(Color col) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, swapR_G(col)); // LED color
        }
        led.setData(buffer); // update to led
        pattern = Optional.empty();
    }

    public void blink(Color color, Time onTime) {
        LEDPattern blinkingColor = LEDPattern.solid(swapR_G(color)); // color of led
        pattern = Optional.of(blinkingColor.blink(onTime));
    }

    public void off() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0); // off
        }
        pattern = Optional.empty();
    }

    /**
     * Swaps the red and green values (from RGB) and returns the new color
     */
    private Color swapR_G(Color col) {
        return new Color(col.green, col.red, col.blue);
    }

    /**
     * LEDs bcome rainbow colored
     */
    public void rainbow(LinearVelocity scrollSpeed) {
        // all hues at maximum saturation and half value
        LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a
        // speed of 1 meter per second.
        LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(scrollSpeed, LEDConstants.kLedSpacing);

        pattern = Optional.of(scrollingRainbow);
    }

    // TODO: relocate code, possibly replace
    public void breathingGradient() {
        // Create an LED pattern that displays a red-to-blue gradient, breathing at a 2 second
        // period (0.5 Hz)
        LEDPattern base =
                LEDPattern.gradient(GradientType.kDiscontinuous, LEDConstants.chargeGold, LEDConstants.chargeGreen);
        LEDPattern pattern = base.breathe(Seconds.of(2));

        // Apply the LED pattern to the data buffer
        pattern.applyTo(buffer);

        // Write the data to the LED strip
        led.setData(buffer);
    }
}
