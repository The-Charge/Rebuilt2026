package frc.robot.commands.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class IdleLED extends Command {

    private final LEDSubsystem ledSub;

    public IdleLED(LEDSubsystem LEDSubsystem) {
        ledSub = LEDSubsystem;
        addRequirements(ledSub);
    }

    @Override
    public void initialize() {
        ledSub.breathe(LEDConstants.chargeGreen, Seconds.of(6));
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        // ledSub.turnOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
