package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class FriendlyZoneLED extends Command {

    private final LEDSubsystem ledSub;
    private final boolean isActive;

    public FriendlyZoneLED(LEDSubsystem LEDSubsystem, boolean isHubActive) {
        ledSub = LEDSubsystem;
        isActive = isHubActive;

        addRequirements(ledSub);
    }

    @Override
    public void initialize() {
        // TODO: alliance zone led logic
        boolean isReadyToShoot = true;

        if (isActive && isReadyToShoot) {
            ledSub.solidColor(LEDConstants.chargeGreen);
        } else if (isActive && !isReadyToShoot) {
            ledSub.solidColor(LEDConstants.orange);
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
