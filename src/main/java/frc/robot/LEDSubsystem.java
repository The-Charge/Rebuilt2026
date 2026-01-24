package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    AddressableLED led;
    AddressableLEDBuffer buffer;

    public LEDSubsystem() {
        led = new AddressableLED(9); // port number
        buffer = new AddressableLEDBuffer(60); // nuumber of LEDs

        led.setLength(buffer.getLength()); // set led buffer
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        update();
    }

    private void update() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kGreen); // LED color
        }
        led.setData(buffer); // update to led
    }
}
