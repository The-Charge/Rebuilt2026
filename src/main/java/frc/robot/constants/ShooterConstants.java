package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants { // split into shooter, spinner, and hood
    public static final int shooterId = -1;
    public static final int hoodId = -1;
    public static final IdleMode idleMode = IdleMode.fromId(-1);
    public static final int currentLimit = -1;
    public static boolean shootInverted = false;
    public static boolean hoodInverted = false;
    public static double shootkP;
    public static double shootkI;
    public static double shootkD;
    public static Rotation2d upAngle;
    public static double hoodTicksPerRadian;
    public static Rotation2d downAngle;
    public static double shootSpeed;

}
