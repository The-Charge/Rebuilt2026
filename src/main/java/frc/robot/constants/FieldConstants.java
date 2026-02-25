package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {
    public static final boolean inTesting = true;

    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    public static final Translation2d redHubLoc = new Translation2d(Inches.of(651.22 - 182.11), Inches.of(158.84));
    public static final Translation2d blueHubLoc = new Translation2d(Inches.of(182.11), Inches.of(158.84));
    public static final Translation2d blueZoneTopLoc =
            new Translation2d(Inches.of(156.61 / 2), Inches.of(317.69 * 3 / 4));
    public static final Translation2d blueZoneBottomLoc =
            new Translation2d(Inches.of(156.61 / 2), Inches.of(317.69 / 4));
    public static final Translation2d redZoneTopLoc =
            new Translation2d(Inches.of(325.61 + 167 + 156.61 / 2), Inches.of(317.69 * 3 / 4));
    public static final Translation2d redZoneBottomLoc =
            new Translation2d(Inches.of(325.61 + 167 + 156.61 / 2), Inches.of(317.69 / 4));

    public static final Distance blueZoneEdge = Inches.of(182.11);
    public static final Distance redZoneEdge = Inches.of(182.11 + 143.5 * 2);
    public static final Distance fieldYCenter = Inches.of(158.84);

    private static final int redHubTag = 10;
    private static final int blueHubTag = 26;

    public static int getHubTag(boolean isRed) {
        if (!inTesting) return 20;
        return isRed ? redHubTag : blueHubTag;
    }

    public static Translation2d getHubLoc(boolean isRed) {
        return isRed ? redHubLoc : blueHubLoc;
    }
}
