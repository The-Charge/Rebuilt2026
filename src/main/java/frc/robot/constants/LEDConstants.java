package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
    public static final String subsystemName = "LED";

    public static final int port = 9;

    // Our LED strip has a density of 20 ICs per meter
    public static final int ledCount = 20 * 3;
    public static final Distance kLedSpacing = Meters.of(1 / 20.0);
    public static final ColorOrder colorOrder = ColorOrder.kRGB;

    public static final Color chargeGreen = new Color("#008800");
    public static final Color chargeGold = new Color("#ffaa00");
    public static final Color orange = new Color("#ff2200");
}
