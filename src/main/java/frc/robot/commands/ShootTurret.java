package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootTurret extends Command {
    private TurretSubsystem turretSub;
    private ShooterSubsystem shooterSub;

    private static final InterpolatingDoubleTreeMap distanceToRPMPlot;

    static {
        distanceToRPMPlot = new InterpolatingDoubleTreeMap();
        distanceToRPMPlot.put(0.0, 0.0);
        distanceToRPMPlot.put(99.0, 99.0);
    }

    public ShootTurret(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem) {
        this.turretSub = turretSubsystem;
        this.shooterSub = shooterSubsystem;

        addRequirements(turretSubsystem);
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double distance = -1; // TODO: Get distance from limelights

        double predictedRPM = distanceToRPMPlot.get(distance); // get power from distance

        shooterSub.shoot(RPM.of(predictedRPM));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
