package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Logger;
import java.util.function.Supplier;

// Set shooter velocity for correct speed based on custom point
public class PrepShootAtPoint extends Command {

    private final ShooterSubsystem shoot;
    private final Supplier<Translation2d> point;

    public PrepShootAtPoint(ShooterSubsystem shootSub, Supplier<Translation2d> pointSupplier) {
        shoot = shootSub;
        point = pointSupplier;

        addRequirements(shoot);
    }

    @Override
    public void execute() {
        Translation2d target = point.get();

        Translation2d offset = target.minus(
                RobotContainer.getInstance().turret.getTurretPoseOnField().getTranslation());
        Distance distToTarget = Meters.of(
                Math.hypot(offset.getMeasureX().in(Meters), offset.getMeasureY().in(Meters)));

        Logger.logDouble(ShooterConstants.subsystemName, "distToTarget", distToTarget.in(Meters));

        shoot.shoot(RPM.of(ShooterConstants.distanceToRPMPlot.get(distToTarget.in(Meters))));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
