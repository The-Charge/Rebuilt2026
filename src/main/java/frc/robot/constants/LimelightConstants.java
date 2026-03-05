package frc.robot.constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class LimelightConstants {
    public static final String subsystemName = "Limelight";

    public static final AngularVelocity kMaxAngularSpeed = DegreesPerSecond.of(720);
    public static final LinearVelocity kMaxSpeedForMegaTag1 = MetersPerSecond.of(0.5);
    public static final Distance kMaxDistanceForMegaTag1 = Meters.of(3.75);
    public static final Distance kMaxDistance = Meters.of(8); // could be increased
    public static final boolean isLoggingVisionDiagnostics = true;

    public static final Time newMegaTag1ReadingThreshold = Seconds.of(10);

    // public static final AprilTagFieldLayout kFieldLayout =
    // AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final LimelightConstants kLimelights[] = {};

    public static final class StdDevConstants {
        public static final class MegaTag1 {
            public static final double kInitialValue = 0.3;
            public static final double kTagCountReward = 0.15;
            public static final double kAverageDistancePunishment = 0.1;
            public static final double kRobotSpeedPunishment = 0.15;
            public static final double kSingleTagPunishment = 0.3;
            public static double kMinStd = 0.05;
        }

        public static final class MegaTag2 {
            public static final double kInitialValue = 0.2;
            public static final double kAverageDistancePunishment = 0.075;
            public static final double kRobotSpeedPunishment = 0.25;
            public static final double kMultipleTagsBonus = 0.05;
            public static double kMinStd = 0.05;
        }
    }

    public enum PoseEstimationMethod {
        MEGATAG_1,
        MEGATAG_2
    }

    public static final AprilTagFieldLayout tagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static double krotStdDev = 0.3;
}
