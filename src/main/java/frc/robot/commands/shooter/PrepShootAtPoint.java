package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;


// Set shooter velocity for correct speed based on custom point
public class PrepShootAtPoint extends Command {

    private final ShooterSubsystem shoot;
    private final SwerveSubsystem swerve;
    private final Supplier<Translation2d> point;

    public PrepShootAtPoint(
            ShooterSubsystem shootSub, SwerveSubsystem swerveSub, Supplier<Translation2d> pointSupplier) {
        shoot = shootSub;
        swerve = swerveSub;
        point = pointSupplier;

        addRequirements(shoot);
    }

    @Override
    public void execute() {
        Translation2d target = point.get();

        Translation2d offset = target.minus(swerve.getPose().getTranslation());
        Distance distToTarget = Meters.of(
                Math.hypot(offset.getMeasureX().in(Meters), offset.getMeasureY().in(Meters)));

        shoot.shoot(RPM.of(ShooterConstants.distanceToRPMPlot.get(distToTarget.in(Meters))));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
