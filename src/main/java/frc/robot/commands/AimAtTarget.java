package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.teleop.TeleopLogic;
import frc.robot.units.TurretAngle;
import frc.robot.utils.Logger;
import java.util.Optional;
import java.util.function.Supplier;

public class AimAtTarget extends Command {

    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final CommandSwerveDrivetrain swerve;
    private final Supplier<Translation2d> targetSupplier;

    private AimAtTarget(
            TurretSubsystem turretSub,
            ShooterSubsystem shooterSub,
            CommandSwerveDrivetrain swerveSub,
            Supplier<Translation2d> target) {
        turret = turretSub;
        shooter = shooterSub;
        addRequirements(turret, shooter);
        swerve = swerveSub;

        targetSupplier = target;
    }

    public static AimAtTarget atHub(
            TurretSubsystem turretSub,
            ShooterSubsystem shooterSub,
            CommandSwerveDrivetrain swerveSub,
            Supplier<Alliance> alliance) {
        return new AimAtTarget(
                turretSub, shooterSub, swerveSub, () -> FieldConstants.getHubLoc(alliance.get() == Alliance.Red));
    }

    public static AimAtTarget atPoint(
            TurretSubsystem turretSub,
            ShooterSubsystem shooterSub,
            CommandSwerveDrivetrain swerveSub,
            Supplier<Translation2d> point) {
        return new AimAtTarget(turretSub, shooterSub, swerveSub, point);
    }

    public static AimAtTarget atFZone(
            TurretSubsystem turretSub,
            ShooterSubsystem shooterSub,
            CommandSwerveDrivetrain swerveSub,
            Supplier<Translation2d> robotLoc) {
        return new AimAtTarget(
                turretSub, shooterSub, swerveSub, () -> TeleopLogic.getFriendlyZoneTarget(robotLoc.get()));
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Translation2d target = targetSupplier.get();
        turret.logTargetPoint(Optional.of(target));

        Pose2d robotPose = swerve.getState().Pose;
        Pose2d turretPose = turret.getTurretPoseOnField();

        Translation2d vectorToTarget = target.minus(turretPose.getTranslation());
        Angle fieldCentricAngle = Radians.of(Math.atan2(vectorToTarget.getY(), vectorToTarget.getX()));
        Angle robotCentricAngle =
                fieldCentricAngle.minus(robotPose.getRotation().getMeasure());

        Distance distToTarget = Meters.of(vectorToTarget.getNorm());

        Logger.logDouble(getName(), "fieldCentricDeg", fieldCentricAngle.in(Degrees));
        Logger.logDouble(ShooterConstants.subsystemName, "distToTarget", distToTarget.in(Meters));

        turret.setTurretAngle(TurretAngle.fromMechanismAngle(robotCentricAngle));
        shooter.setTargetVelocity(RPM.of(ShooterConstants.distanceToRPMPlot.get(distToTarget.in(Meters))));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
