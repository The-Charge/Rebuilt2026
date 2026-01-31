package frc.robot.constants;

import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import java.util.Optional;

public class IntakeConstants {
    public static final String name = "Intake";

    public static class Roller {
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = false;
        public static final double maxDutyCycle = 1.00;
        public static final Optional<Double> nominalVoltage = Optional.empty();
        public static final int currentLimit = 20;
        public static final double speed = 1.00;
        public static final int motorID = 9;
        public static final double overheatingTemp = 80; // celcius
    }

    public static class Servo {
        public static final ChannelId channelId = ChannelId.kChannelId0;
        public static final int deployedPulseWidth = 2500; // in microseconds for pulsewidth [500, 2500]
        public static final int servoID = 15;
    }
}
