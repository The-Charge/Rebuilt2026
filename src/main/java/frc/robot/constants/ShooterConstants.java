package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Optional;

public class ShooterConstants { // split into shooter, spinner, and hood
    // public static final double hoodPosThreshold = 3;
    // public static Rotation2d upAngle;
    // public static Rotation2d downAngle;

    public static class ShootConfig {
        public static final double maxDutyCycle = 1.0;
        public static final Optional<Double> nominalVoltage = Optional.empty();

        public static final int motorID = -1;

        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static final Optional<Double> kStaticG = Optional.empty();
        public static final Optional<Double> kCos = Optional.empty();
        public static final Optional<Double> kS = Optional.empty();
        public static final Optional<Double> kV = Optional.empty();
        public static final Optional<Double> kA = Optional.empty();

        public static boolean inverted = false;
        public static int currentLimit = 20;
        public static IdleMode idleMode = IdleMode.kCoast;

        public static final AngularVelocity targetTolerance = RPM.of(100);
    }

    // public static class HoodConfig {
    //     public static int ID = 1000;

    //     public static double p = 0;
    //     public static double i = 0;
    //     public static double d = 0;

    //     public static boolean inverted = false;
    //     public static int currentLimit = 0;
    //     public static IdleMode idleMode = IdleMode.kBrake;

    //     public static double ticksPerRadian;
    // }

    public static final InterpolatingDoubleTreeMap distanceToRPMPlot;

    static {
        distanceToRPMPlot = new InterpolatingDoubleTreeMap();
        distanceToRPMPlot.put(0.0, 0.0);
        distanceToRPMPlot.put(69.0, 69.0);
    }

    public static final String subsystemName = "Shooter";
}
