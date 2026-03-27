package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {

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

    public static Translation2d getHubLoc(boolean isRed) {
        return isRed ? redHubLoc : blueHubLoc;
    }

    public static Translation2d getFriendlyZoneTarget(Translation2d robotPose) {
        boolean isInTopHalf = robotPose.getMeasureY().gt(FieldConstants.fieldYCenter);
        boolean isBlue =
                DriverStation.getAlliance().map((val) -> val == Alliance.Blue).orElse(true);

        if (isBlue) {
            if (isInTopHalf) {
                return FieldConstants.blueZoneTopLoc;
            } else {
                return FieldConstants.blueZoneBottomLoc;
            }
        } else {
            if (isInTopHalf) {
                return FieldConstants.redZoneTopLoc;
            } else {
                return FieldConstants.redZoneBottomLoc;
            }
        }
    }
}
