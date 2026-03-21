package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
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
            CommandSwerveDrivetrain noDepSwerveSub,
            Supplier<Translation2d> target) {
        turret = turretSub;
        shooter = shooterSub;
        addRequirements(turret, shooter);
        swerve = noDepSwerveSub;

        targetSupplier = target;
    }

    public static AimAtTarget atHub(
            TurretSubsystem turretSub, ShooterSubsystem shooterSub, CommandSwerveDrivetrain noDepSwerveSub) {
        return new AimAtTarget(
                turretSub,
                shooterSub,
                noDepSwerveSub,
                () -> FieldConstants.getHubLoc(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red));
    }

    public static AimAtTarget atPoint(
            TurretSubsystem turretSub,
            ShooterSubsystem shooterSub,
            CommandSwerveDrivetrain noDepSwerveSub,
            Supplier<Translation2d> point) {
        return new AimAtTarget(turretSub, shooterSub, noDepSwerveSub, point);
    }

    public static AimAtTarget atFZone(
            TurretSubsystem turretSub, ShooterSubsystem shooterSub, CommandSwerveDrivetrain noDepSwerveSub) {
        return new AimAtTarget(
                turretSub,
                shooterSub,
                noDepSwerveSub,
                () -> TeleopLogic.getFriendlyZoneTarget(
                        noDepSwerveSub.getState().Pose.getTranslation()));
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Translation2d target = targetSupplier.get();
        turret.logTargetPoint(Optional.of(target));

        ChassisSpeeds botSpeeds = RobotContainer.getInstance().swerve.getState().Speeds;
        Translation2d drivebyBallDisplacement = new Translation2d(
                botSpeeds.vxMetersPerSecond * ShooterConstants.ballAirTime.in(Seconds),
                botSpeeds.vyMetersPerSecond * ShooterConstants.ballAirTime.in(Seconds));

        target = target.minus(drivebyBallDisplacement);

        Pose2d robotPose = swerve.getState().Pose;
        Pose2d turretPose = turret.getTurretPoseOnField();

        Translation2d vectorToTarget = target.minus(turretPose.getTranslation());
        Angle fieldCentricAngle = Radians.of(Math.atan2(vectorToTarget.getY(), vectorToTarget.getX()));
        Angle robotCentricAngle =
                fieldCentricAngle.minus(robotPose.getRotation().getMeasure());

        Distance distToTarget = Meters.of(vectorToTarget.getNorm());

        Logger.logDouble(getName(), "fieldCentricDeg", fieldCentricAngle.in(Degrees));
        Logger.logDouble(getName(), "distToTarget", distToTarget.in(Meters));

        turret.setTurretAngle(TurretAngle.fromMechanismAngle(robotCentricAngle));
        shooter.setTargetVelocity(RPM.of(ShooterConstants.distanceToRPMPlot.get(distToTarget.in(Meters))));
    }

    @Override
    public void end(boolean interrupted) {
        turret.logTargetPoint(Optional.empty());
        Logger.logDouble(getName(), "fieldCentricDeg", Double.NaN);
        Logger.logDouble(getName(), "distToTarget", Double.NaN);

        turret.stopTurret();
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
