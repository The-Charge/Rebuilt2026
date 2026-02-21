package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.BooleanSupplier;

public class ActiveAtHubLED extends Command {

    private final LEDSubsystem ledSub;
    private final BooleanSupplier isReady;

    public ActiveAtHubLED(LEDSubsystem LEDSubsystem, BooleanSupplier isReadyToShoot) {
        ledSub = LEDSubsystem;
        isReady = isReadyToShoot;

        addRequirements(ledSub);
    }

    @Override
    public void initialize() {
        if (isReady.getAsBoolean()) {
            ledSub.solidColor(LEDConstants.chargeGreen);
        } else {
            ledSub.solidColor(LEDConstants.orange);
        }
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
        return false;
    }
}
