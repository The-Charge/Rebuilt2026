package frc.robot.constants;

import com.revrobotics.servohub.ServoChannel.ChannelId;

public class IntakeConstants {
    public static int RollerMotorId = 9;
    public static int deployerServoID = 3;

    public static double DeployedPosition = 2000;
    public static double RetractedPosition = 0;

    public static double StateTolerance = 500;
    public static int currentLimit = 20;

    public static ChannelId channelId = ChannelId.kChannelId0;
    public static int deployedPulseWidth = 2500; // in microseconds for pulsewidth
}
