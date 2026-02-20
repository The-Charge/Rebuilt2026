package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;
import java.util.function.BooleanSupplier;

public class FriendlyZoneLED extends Command {

    private final LEDSubsystem ledSub;
    private final BooleanSupplier isActive;
    private final BooleanSupplier isReady;

    public FriendlyZoneLED(LEDSubsystem LEDSubsystem, BooleanSupplier isHubActive, BooleanSupplier isReadyToShoot) {
        ledSub = LEDSubsystem;
        isActive = isHubActive;
        isReady = isReadyToShoot;

        addRequirements(ledSub);
    }

    @Override
    public void initialize() {
        if (isActive.getAsBoolean()) {
            if (isReady.getAsBoolean()) {
                ledSub.solidColor(LEDConstants.chargeGreen);
            } else {
                ledSub.solidColor(LEDConstants.orange);
            }
        } else {
            ledSub.solidColor(Color.kWhite);
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
