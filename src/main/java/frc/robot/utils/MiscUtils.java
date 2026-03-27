package frc.robot.utils;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.Status;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MiscUtils {

    private MiscUtils() {}

    public static void changeSubsystemDefaultCommand(Subsystem sub, Command newDefault, boolean cancelExistingCommand) {
        if (sub == null) return;

        Command currentDefault = sub.getDefaultCommand();
        Command currentCommand = sub.getCurrentCommand();

        if (currentCommand != null) {
            if (cancelExistingCommand) {
                currentCommand.cancel();
            } else if (currentDefault != null && currentDefault.getClass().equals(currentCommand.getClass())) {
                // only cancel current command if it is the previous default command
                currentCommand.cancel();
            }
        }
        sub.setDefaultCommand(newDefault);
    }

    public static void removeSubsystemDefaultCommand(Subsystem sub, boolean cancelExistingCommand) {
        if (sub == null) return;

        Command currentDefault = sub.getDefaultCommand();
        Command currentCommand = sub.getCurrentCommand();

        sub.removeDefaultCommand();
        if (currentCommand != null) {
            if (cancelExistingCommand) {
                currentCommand.cancel();
            } else if (currentDefault != null && currentDefault.getClass().equals(currentCommand.getClass())) {
                currentCommand.cancel();
            }
        }
    }

    public static boolean isPDPConnected(PowerDistribution pdp) {
        if (pdp == null) return false;
        return pdp.getVoltage() != 0;
    }

    public static boolean criticalTOFState(TimeOfFlight tof) {
        if (tof == null) return false;

        Status status = tof.getStatus();
        return status == Status.HardwareFailure || status == Status.InternalError || status == Status.WrappedTarget;
    }
}
