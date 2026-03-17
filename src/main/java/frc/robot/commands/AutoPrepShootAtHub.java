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
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.units.TurretAngle;
import frc.robot.utils.Logger;
import java.util.Optional;
import java.util.function.Supplier;

public class AutoPrepShootAtHub extends Command {

    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final Supplier<Alliance> alliance;

    public AutoPrepShootAtHub(
            ShooterSubsystem shooterSub, TurretSubsystem turretSub, Supplier<Alliance> allianceSupplier) {
        shooter = shooterSub;
        turret = turretSub;
        alliance = allianceSupplier;

        addRequirements(shooter, turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = RobotContainer.getInstance().swerve.getStateCopy().Pose;

        Translation2d hubLoc = FieldConstants.getHubLoc(alliance.get() == Alliance.Red);
        Translation2d offsetToHub = hubLoc.minus(
                RobotContainer.getInstance().turret.getTurretPoseOnField().getTranslation());
        Distance distToTarget = Meters.of(Math.hypot(
                offsetToHub.getMeasureX().in(Meters), offsetToHub.getMeasureY().in(Meters)));

        Angle fieldCentricAngle = Radians.of(Math.atan2(offsetToHub.getY(), offsetToHub.getX()));
        Angle robotCentricAngle =
                fieldCentricAngle.minus(robotPose.getRotation().getMeasure());

        Logger.logDouble(ShooterConstants.subsystemName, "distToTarget", distToTarget.in(Meters));
        Logger.logDouble("AlignTurret", "fieldCentricAngle", fieldCentricAngle.in(Degrees));
        turret.logTargetPoint(Optional.of(hubLoc));

        shooter.setTargetVelocity(RPM.of(ShooterConstants.distanceToRPMPlot.get(distToTarget.in(Meters))));
        turret.setTurretAngle(TurretAngle.fromMechanismAngle(robotCentricAngle));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
