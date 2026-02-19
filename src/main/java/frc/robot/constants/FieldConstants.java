package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final boolean onField = true;

    private static final Pose2d redHubPos = new Pose2d(
            Meters.convertFrom(651.22 - 182.11, Inches), Meters.convertFrom(158.84, Inches), new Rotation2d());
    private static final Pose2d blueHubPos =
            new Pose2d(Meters.convertFrom(182.11, Inches), Meters.convertFrom(158.84, Inches), new Rotation2d());
    public static final double hubHeight = -1;
    public static final Pose2d redAllianceLeft = new Pose2d(
            Meters.convertFrom(650.12 - 182.11 / 2, Inches), Meters.convertFrom(158.32 / 2, Inches), new Rotation2d());
    public static final Pose2d redAllianceRight = new Pose2d(
            Meters.convertFrom(650.12 - 182.11 / 2, Inches),
            Meters.convertFrom(316.64 - 158.32 / 2, Inches),
            new Rotation2d());
    public static final Pose2d blueAllianceLeft = new Pose2d(
            Meters.convertFrom(182.11 / 2, Inches), Meters.convertFrom(316.64 - 158.32 / 2, Inches), new Rotation2d());
    public static final Pose2d blueAllianceRight = new Pose2d(
            Meters.convertFrom(182.11 / 2, Inches), Meters.convertFrom(158.32 / 2, Inches), new Rotation2d());

    private static final int redHubTag = 10;
    private static final int blueHubTag = 26;

    public static int getHubTag(boolean isRed) {
        if (!onField) return 20;
        return isRed ? redHubTag : blueHubTag;
    }

    public static Pose2d getHubPos(boolean isRed) {
        return isRed ? redHubPos : blueHubPos;
    }
}
