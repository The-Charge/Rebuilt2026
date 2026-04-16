package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import java.util.Optional;

public class ShooterConstants {

    public static final String subsystemName = "Shooter";

    public static class Motor {
        public static final int motorID = 4;
        public static final String motorName = "shooterMotor";

        public static Current currentLimit = Amps.of(40);
        public static IdleMode idleMode = IdleMode.kCoast;
        public static boolean inverted = false;
        public static final double maxDutyCycle = 1.0;
        public static final Optional<Voltage> nominalVoltage = Optional.empty();

        public static class NormalPID {
            public static double kP = 9.3539e-4;
            public static double kI = 0;
            public static final Optional<Double> iZone = Optional.empty();
            public static double kD = 0;
            public static final Optional<Voltage> kStaticG = Optional.empty();
            public static final Optional<Voltage> kCos = Optional.empty();
            public static final Optional<Double> kS = Optional.of(0.11245);
            public static final Optional<Double> kV = Optional.of(12.071 / 6756);
            public static final Optional<Double> kA = Optional.of(0.036117);
            public static final Optional<Time> rampTime = Optional.empty();
        }

        public static class BackupPID {
            public static double kP = 0.001;
            public static double kI = 5e-7;
            public static final Optional<Double> iZone = Optional.of(2000d);
            public static double kD = 4e-4;
            public static final Optional<Voltage> kStaticG = Optional.empty();
            public static final Optional<Voltage> kCos = Optional.empty();
            public static final Optional<Double> kS = Optional.empty();
            public static final Optional<Double> kV = Optional.empty();
            public static final Optional<Double> kA = Optional.empty();
            public static final Optional<Time> rampTime = Optional.empty();
        }
    }

    public static final AngularVelocity targetUpwardTolerance = RPM.of(100);
    public static final AngularVelocity targetDownwardTolerance = RPM.of(100);
    public static final AngularVelocity maxManualSpeed = RPM.of(7000);

    public static final boolean manualShootUseSmartdashboard = true;

    public static final Time ballAirTime =
            Seconds.of(0.8); // This value is quite consistent across distances, and obtained from footage
    public static final InterpolatingDoubleTreeMap distanceToRPMPlot;

    static {
        // https://docs.google.com/spreadsheets/d/1gD5zpmwW_OerKw1IJ8A7G90_bQRWeTSE9Q0T1-Z4_QU/edit?usp=sharing
        distanceToRPMPlot = new InterpolatingDoubleTreeMap();

        distanceToRPMPlot.put(1.96, 2400d);
        distanceToRPMPlot.put(2.50, 2550d);
        distanceToRPMPlot.put(3.03, 2700d);
        distanceToRPMPlot.put(3.31, 2900d);
        distanceToRPMPlot.put(4.17, 3200d);
        distanceToRPMPlot.put(4.37, 3300d);
    }
}
