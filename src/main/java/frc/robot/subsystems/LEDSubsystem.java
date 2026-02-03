package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private AddressableLEDBuffer buffer;
    private LEDPattern pattern;

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.port); // port number
        buffer = new AddressableLEDBuffer(LEDConstants.ledCount); // nuumber of LEDs
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
        pattern = null;
    }

    @Override
    public void periodic() {
        led.setData(buffer);
        if (pattern != null) {
            pattern.applyTo(buffer);
        }
    }

    public void turnGreen() {
        for (int i = 0; i < buffer.getLength(); i++) {
            setLEDColor(i, LEDConstants.chargeGreen); // LED color
        }
        led.setData(buffer); // update to led
        pattern = null;
    }

    public void blink(Color color) {
        LEDPattern blinkingColor = LEDPattern.solid(swapR_G(color)); // color of led
        pattern = blinkingColor.blink(Seconds.of(LEDConstants.blinkInterval));
    }

    public void turnOff() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0); // off
        }
        pattern = null;
    }

    /**
     * swap green and red to account for incorrect LED type
     * @param index
     * @param col
     */
    private void setLEDColor(int index, Color col) {
        buffer.setLED(index, new Color(col.green, col.red, col.blue));
    }

    /**
     * Swaps the red and green values (from RGB) and returns the new color
     */
    private Color swapR_G(Color col) {
        return (new Color(col.green, col.red, col.blue));
    }

    /**
     * LEDs bcome rainbow colored
     */
    public void rainbow() {
        // all hues at maximum saturation and half brightness
        LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a
        // speed of 1 meter per second.
        LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LEDConstants.kLedSpacing);
        scrollingRainbow.applyTo(buffer);
        led.setData(buffer);
    }

    public void idleLeds() {
        // Create an LED pattern that displays a red-to-blue gradient, breathing at a 2 second
        // period (0.5 Hz)
        LEDPattern base = LEDPattern.discontinuousGradient(LEDConstants.chargeGold, LEDConstants.chargeGreen);
        LEDPattern pattern = base.breathe(Seconds.of(2));

        // Apply the LED pattern to the data buffer
        pattern.applyTo(buffer);

        // Write the data to the LED strip
        led.setData(buffer);
    }

    public void allianceZone() {}

    public void neutralZone() {}
}
