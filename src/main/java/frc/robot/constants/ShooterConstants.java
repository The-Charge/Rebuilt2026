package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants { // split into shooter, spinner, and hood

    public static Rotation2d upAngle;
    public static Rotation2d downAngle;

    public static class ShootConfig {
        public static int ID;

        public static double p;
        public static double i;
        public static double d;

        public static boolean inverted;
        public static int currentLimit;
        public static IdleMode idleMode;
    }

    public static class HoodConfig {
        public static int ID;

        public static double p;
        public static double i;
        public static double d;

        public static boolean inverted;
        public static int currentLimit;
        public static IdleMode idleMode;

        public static double ticksPerRadian;
    }
}
