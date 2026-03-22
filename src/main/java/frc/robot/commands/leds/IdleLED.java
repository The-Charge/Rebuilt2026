package frc.robot.commands.leds;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class IdleLED extends Command {

    private final LEDSubsystem led;

    public IdleLED(LEDSubsystem ledSub) {
        led = ledSub;
        addRequirements(led);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void initialize() {
        led.breathe(LEDConstants.chargeGold, Seconds.of(6));
    }

    @Override
    public void end(boolean interrupted) {
        led.off();
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
