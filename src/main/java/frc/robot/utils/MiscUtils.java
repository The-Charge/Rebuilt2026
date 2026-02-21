package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MiscUtils {

    private MiscUtils() {}

    public static void changeSubsystemDefaultCommand(Subsystem sub, Command newDefault, boolean force) {
        if (sub == null) return;

        Command currentDefault = sub.getDefaultCommand();
        Command currentCommand = sub.getCurrentCommand();

        if (currentCommand != null) {
            if (force) {
                currentCommand.cancel();
            } else if (currentDefault != null && currentDefault.getClass().equals(currentCommand.getClass())) {
                // only cancel current command if it is the previous default command
                currentCommand.cancel();
            }
        }
        sub.setDefaultCommand(newDefault);
    }
}
