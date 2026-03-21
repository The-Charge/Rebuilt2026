package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class ShooterConstants {

    public static final String subsystemName = "Shooter";

    public static final int motorID = 4;
    public static final double maxDutyCycle = 1.0;
    public static final Optional<Double> nominalVoltage = Optional.empty();
    public static double kP = 0.001;
    public static double kI = 5e-7;
    public static double kD = 4e-4;
    public static final Optional<Voltage> kStaticG = Optional.empty();
    public static final Optional<Voltage> kCos = Optional.empty();
    public static final Optional<Voltage> kS = Optional.empty();
    public static final Optional<Voltage> kV = Optional.empty();
    public static final Optional<Voltage> kA = Optional.empty();
    public static final Optional<Double> iZone = Optional.of(2000d);

    public static boolean inverted = false;
    public static int currentLimit = 40;
    public static IdleMode idleMode = IdleMode.kCoast;

    public static final AngularVelocity targetTolerance = RPM.of(100);
    public static final AngularVelocity maxManualSpeed = RPM.of(7000);

    public static final boolean manualShootUseSmartdashboard = true;
    public static final Time ballAirTime =
            Seconds.of(0.7); // This value is quite consistent across distances, and obtained from footage

    public static final InterpolatingDoubleTreeMap distanceToRPMPlot;

    static {
        distanceToRPMPlot = new InterpolatingDoubleTreeMap();

        distanceToRPMPlot.put(1.9505, 2500.0d);
        distanceToRPMPlot.put(2.2, 2530.0d);
        distanceToRPMPlot.put(2.860123, 2900.0d);
        distanceToRPMPlot.put(3.084457, 2700.0d);
        distanceToRPMPlot.put(3.203391, 2800.0d);
        distanceToRPMPlot.put(3.40251, 2900.0d);
        distanceToRPMPlot.put(3.672000, 3000.0d);
        distanceToRPMPlot.put(4.136633, 3100.0d);
        distanceToRPMPlot.put(4.876027, 3500.0d);
        distanceToRPMPlot.put(5.181369, 3700.0d);
        distanceToRPMPlot.put(5.610685, 4000.0d);
    }
}
