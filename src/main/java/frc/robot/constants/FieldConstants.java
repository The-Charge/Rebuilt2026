package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d redHubPos = new Pose2d(
            Meters.convertFrom(651.22 - 182.11, Inches), Meters.convertFrom(158.84, Inches), new Rotation2d());
    public static final Pose2d blueHubPos =
            new Pose2d(Meters.convertFrom(182.11, Inches), Meters.convertFrom(158.84, Inches), new Rotation2d());
    public static final double hubHeight = -1;
}
