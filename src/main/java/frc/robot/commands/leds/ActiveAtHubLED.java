package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.BooleanSupplier;

public class ActiveAtHubLED extends Command {

    private final LEDSubsystem led;
    private final BooleanSupplier isReady;

    public ActiveAtHubLED(LEDSubsystem ledSub, BooleanSupplier isReadyToShoot) {
        led = ledSub;
        isReady = isReadyToShoot;

        addRequirements(led);
    }

    @Override
    public String getName() {
        return getClass().getTypeName();
    }

    @Override
    public void execute() {
        if (isReady.getAsBoolean()) {
            led.solidColor(LEDConstants.chargeGreen);
        } else {
            led.solidColor(LEDConstants.orange);
        }
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
        return false;
    }
}
