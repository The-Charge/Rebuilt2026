package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    AddressableLED led;
    AddressableLEDBuffer buffer;

    public static int bufferLength = 60;

    public LEDSubsystem() {
        led = new AddressableLED(9); // port number
        buffer = new AddressableLEDBuffer(bufferLength); // nuumber of LEDs

        led.setLength(buffer.getLength()); // set led buffer
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        update();
    }

    private void update() {
        //
    }

    public void turnGreen() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, chargeGreen); // LED color
        }
        led.setData(buffer); // update to led
    }

    public void turnOff() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0); // off
        }
        led.setData(buffer);
    }
}
