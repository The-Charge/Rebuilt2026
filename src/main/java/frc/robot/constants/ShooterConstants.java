package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class ShooterConstants { // split into shooter, spinner, and hood
    // public static final double hoodPosThreshold = 3;
    // public static Rotation2d upAngle;
    // public static Rotation2d downAngle;

    public static class ShootConfig {
        public static final double maxDutyCycle = 1.0;
        public static final Optional<Double> nominalVoltage = Optional.empty();

        public static final int motorID = 4;

        public static double kP = 0.001;
        public static double kI = 5e-7;
        public static double kD = 0;
        public static final Optional<Voltage> kStaticG = Optional.empty();
        public static final Optional<Voltage> kCos = Optional.empty();
        public static final Optional<Voltage> kS = Optional.empty();
        public static final Optional<Voltage> kV = Optional.empty();
        public static final Optional<Voltage> kA = Optional.empty();
        public static final Optional<Double> iZone = Optional.of(2000d);

        public static boolean inverted = false;
        public static int currentLimit = 30;
        public static IdleMode idleMode = IdleMode.kCoast;

        public static final AngularVelocity targetTolerance = RPM.of(100);
        public static final AngularVelocity maxManualSpeed = RPM.of(7000);
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
        distanceToRPMPlot.put(2.116197, 2500.0d); // not that reliable
        distanceToRPMPlot.put(2.2, 2530.0d); 
        distanceToRPMPlot.put(2.291396, 2570.0d); 
        distanceToRPMPlot.put(2.531513, 2600.0d);
        // 2.7
        distanceToRPMPlot.put(3.084457, 2700.0d);
        distanceToRPMPlot.put(3.203391, 2800.0d);
        distanceToRPMPlot.put(3.40251, 2900.0d);
        distanceToRPMPlot.put(3.672000, 3000.0d);
        // 3.9
        distanceToRPMPlot.put(4.136633, 3100.0d);
        // 4.3
        // 4.5
        distanceToRPMPlot.put(4.876027, 3500.0d);
        distanceToRPMPlot.put(5.181369, 3700.0d);
        // 5.4
        distanceToRPMPlot.put(5.610685, 4000.0d);


    }

    public static final String subsystemName = "Shooter";
}
