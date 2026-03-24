package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import limelight.networktables.LimelightSettings.ImuMode;

public class LimelightConstants {
    public static final String subsystemName = "Limelight";

    public static class Turret {
        public static final String cameraName = "turret";
        public static final Pose3d robotRelativePose = new Pose3d(
                Inches.of(-27 / 2 + 5 / 8),
                Inches.of(27 / 2 - 5.5),
                Inches.of(8.75),
                new Rotation3d(Degrees.of(0), Degrees.of(30), Degrees.of(180)));
    }

    public static class Side {
        public static final String cameraName = "other";
        public static final Pose3d robotRelativePose = new Pose3d(
                Inches.of(-27 / 2 + 5.5),
                Inches.of(-27 / 2 + 1),
                Inches.of(8 + 5 / 8),
                new Rotation3d(Degrees.of(0), Degrees.of(30), Degrees.of(90)));
    }

    public static final AngularVelocity kMaxAngularSpeed = DegreesPerSecond.of(720);
    public static final LinearVelocity kMaxSpeedForMegaTag1 = MetersPerSecond.of(0.5);
    public static final Distance kMaxDistanceForMegaTag1 = Meters.of(3.75);
    public static final Distance kMaxDistance = Meters.of(8); // could be increased

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
    public static final int aprilTagPipelineIndex = 1;
    public static final int throttlePipelineIndex = 0;

    public static double kRotStdDev = 0.3;

    public static double imuAssistAlpha = 0.1;

    public static ImuMode imuMode = ImuMode.InternalImuMT1Assist;
}
