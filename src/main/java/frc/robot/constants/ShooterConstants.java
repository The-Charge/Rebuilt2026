package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants { // split into shooter, spinner, and hood
    public static Rotation2d upAngle;
    public static Rotation2d downAngle;

    public static class ShootConfig {
        public static final double maxDutyCycle = 0;

        public static int ID = -1;

        public static double p = 0;
        public static double i = 0;
        public static double d = 0;

        public static boolean inverted = false;
        public static int currentLimit = 20;
        public static IdleMode idleMode = IdleMode.kCoast;
    }

    public static class HoodConfig {
        public static int ID = 9;

        public static double p = 0;
        public static double i = 0;
        public static double d = 0;

        public static boolean inverted = false;
        public static int currentLimit = 0;
        public static IdleMode idleMode = IdleMode.kBrake;

        public static double ticksPerRadian;
    }

    public static final InterpolatingDoubleTreeMap distanceToRPMPlot;

    static {
        distanceToRPMPlot = new InterpolatingDoubleTreeMap();
        distanceToRPMPlot.put(0.0, 0.0);
        distanceToRPMPlot.put(99.0, 99.0);
    }

    public static final String subsystemName = "Shooter";
}
