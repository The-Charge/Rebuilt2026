package frc.robot.subsystems;

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

    private void setLEDColor(int index, Color col) {
        // swap green and red to account for incorrect LED type
        buffer.setLED(index, new Color(col.green, col.red, col.blue));
    }

    /**
     * Swaps the red and green values (from RGB) and returns the new color
     */
    private Color swapR_G(Color col) {
        return (new Color(col.green, col.red, col.blue));
    }
}
