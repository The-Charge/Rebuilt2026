package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private AddressableLEDBuffer buffer;

    public static int bufferLength = 60;

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.port); // port number
        buffer = new AddressableLEDBuffer(LEDConstants.ledCount); // nuumber of LEDs

        led.setLength(buffer.getLength()); // set led buffer
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        led.setData(buffer);
    }

    public void turnGreen() {
        for (int i = 0; i < buffer.getLength(); i++) {
            setLEDColor(i, i % 2 == 0 ? LEDConstants.chargeGreen : LEDConstants.chargeGold); // LED color
        }
        led.setData(buffer); // update to led
    }

    public void turnOff() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0); // off
        }
    }

    private void setLEDColor(int index, Color col) {
        // swap green and red to account for incorrect LED type
        buffer.setLED(index, new Color(col.green, col.red, col.blue));
    }
}
