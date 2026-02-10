package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootTurret extends Command {
    private final ShooterSubsystem shooterSub;

    public ShootTurret(ShooterSubsystem shooterSubsystem) {
        shooterSub = shooterSubsystem;

        addRequirements(shooterSub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double distance = -1; // TODO: Get distance from limelights

        double predictedRPM = ShooterConstants.distanceToRPMPlot.get(distance); // get power from distance

        shooterSub.shoot(RPM.of(predictedRPM));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
