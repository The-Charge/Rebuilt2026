package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
    public static final int port = 0;
    public static final int ledCount = 43;
    public static final int blinkInterval = 1; // in seconds

    public static final Color chargeGreen = new Color("#008800");
    public static final Color chargeGold = new Color("#ffaa00");
    public static final Color orange = new Color("#ff2200");
    public static final Color white = new Color("#ffffff");

    // Our LED strip has a density of 120 LEDs per meter
    public static final Distance kLedSpacing = Meters.of(1 / 120.0);
}
