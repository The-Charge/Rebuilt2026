package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import java.util.Optional;

public class TurretConstants { // split into shooter, spinner, and hood
    public static final int hoodChannel = 17;

    public static final int shooterMotorId = 813;
    public static final int shooterId = 914;
    public static final IdleMode idleMode = IdleMode.fromId(0);
    public static final int currentLimit = 40;
    public static final boolean inverted = false;
    public static double shooterAcceptableAngle;
    public static PIDController shooterPID = new PIDController(0, 0, 0);

    public class Spin {
        public static final int motorID = 9;
        public static final int maxCurrent = 10;
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = false;
        public static final double maxDutyCycle = .5;
        public static final Optional<Double> nominalVoltage = Optional.empty();
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final Optional<Double> kStaticG = Optional.empty();
        public static final Optional<Double> kCos = Optional.empty();
        public static final Optional<Double> kS = Optional.empty();
        public static final Optional<Double> kV = Optional.empty();
        public static final Optional<Double> kA = Optional.empty();

        public static final double ticksPerRadian = 3;
        public static final double radiansPerTick = 1 / ticksPerRadian;
    }
}
